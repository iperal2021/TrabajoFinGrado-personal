#!/usr/bin/env python3
"""Servidor del monitor web de marcha.

Sirve ros2_gait_monitor.html (sustituye a `python3 -m http.server`) y anade
una API JSON minima para la carpeta de registros de pacientes, ya que el
navegador no puede tocar el sistema de ficheros:

  GET  /api/records/status  -> carpeta de registros activa (o la propuesta)
  POST /api/records/create  -> crea <Documentos>/patient_records
  POST /api/records/select  -> usa una carpeta elegida con el explorador
  GET  /api/fs/list?path=   -> subcarpetas de `path` (explorador "Browse")
  POST /api/session/start   -> crea <registros>/<id_paciente>/ y da el sello
  POST /api/session/notes   -> escribe <id>_<pre|post>_<sello>.txt

Solo stdlib: start.sh expulsa cualquier venv antes de lanzar nada.
"""

import argparse
import datetime
import json
import os
from pathlib import Path
import re
import subprocess
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

SCRIPT_DIR = Path(__file__).resolve().parent
RECORDS_NAME = 'patient_records'
CONFIG_FILE = (
    Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config'))
    / 'gait_monitor' / 'config.json')
# El ID se usa como nombre de carpeta/fichero: nada de '/', '..' ni ocultos.
PATIENT_ID_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$')
STAMP_RE = re.compile(r'^\d{8}_\d{6}$')
STAMP_FMT = '%Y%m%d_%H%M%S'
PHASES = ('pre', 'post')
# El explorador solo navega por aqui (home + discos externos montados).
BROWSE_ROOTS = [Path.home(), Path('/media'), Path('/mnt')]


class ApiError(Exception):

    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def documents_dir():
    """Carpeta de documentos del usuario, respetando la localizacion
    (p. ej. ~/Documentos en un Ubuntu en espanol)."""
    try:
        out = subprocess.run(
            ['xdg-user-dir', 'DOCUMENTS'], capture_output=True, text=True,
            timeout=2).stdout.strip()
        if out and Path(out) != Path.home():
            return Path(out)
    except (OSError, subprocess.SubprocessError):
        pass
    return Path.home() / 'Documents'


def load_config():
    try:
        return json.loads(CONFIG_FILE.read_text())
    except (OSError, ValueError):
        return {}


def save_config(cfg):
    CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
    CONFIG_FILE.write_text(json.dumps(cfg, indent=2))


def find_similar(docs):
    """Carpetas de Documentos que parecen un registro de pacientes
    (Patient_Records, pacientes, registro-pacientes...)."""
    if not docs.is_dir():
        return []
    similar = []
    for d in sorted(docs.iterdir()):
        if d.is_dir() and not d.name.startswith('.'):
            norm = re.sub(r'[^a-z]', '', d.name.lower())
            if 'patient' in norm or 'paciente' in norm:
                similar.append(d)
    return similar


def records_status():
    # Orden: carpeta elegida antes (config) > patient_records > similar.
    configured = load_config().get('records_dir')
    if configured and Path(configured).is_dir():
        return {'found': True, 'path': configured, 'source': 'config'}
    docs = documents_dir()
    default = docs / RECORDS_NAME
    if default.is_dir():
        return {'found': True, 'path': str(default), 'source': 'default'}
    similar = find_similar(docs)
    if similar:
        return {'found': True, 'path': str(similar[0]), 'source': 'similar'}
    return {'found': False, 'default_path': str(default)}


def records_dir():
    status = records_status()
    if not status['found']:
        raise ApiError(HTTPStatus.CONFLICT, 'No hay carpeta de registros')
    return Path(status['path'])


def is_browsable(path):
    return any(path == r or r in path.parents for r in BROWSE_ROOTS)


def use_records_dir(path):
    if not path.is_dir() or not os.access(path, os.W_OK):
        raise ApiError(HTTPStatus.BAD_REQUEST, f'Carpeta no escribible: {path}')
    cfg = load_config()
    cfg['records_dir'] = str(path)
    save_config(cfg)
    return {'path': str(path)}


def api_create(_body):
    path = documents_dir() / RECORDS_NAME
    path.mkdir(parents=True, exist_ok=True)
    return use_records_dir(path)


def api_select(body):
    path = Path(str(body.get('path', ''))).expanduser().resolve()
    if not is_browsable(path):
        raise ApiError(HTTPStatus.FORBIDDEN, f'Ruta no permitida: {path}')
    return use_records_dir(path)


def api_list(query):
    raw = query.get('path', [''])[0]
    if raw:
        path = Path(raw).expanduser().resolve()
    else:
        status = records_status()
        path = Path(status['path']) if status['found'] else documents_dir()
        if not path.is_dir():
            path = Path.home()
    if not is_browsable(path) or not path.is_dir():
        raise ApiError(HTTPStatus.FORBIDDEN, f'Ruta no permitida: {path}')
    try:
        dirs = sorted(
            (d for d in path.iterdir()
             if d.is_dir() and not d.name.startswith('.')),
            key=lambda d: d.name.lower())
    except PermissionError:
        dirs = []
    parent = path.parent if is_browsable(path.parent) and path.parent != path else None
    return {
        'path': str(path),
        'parent': str(parent) if parent else None,
        'dirs': [{'name': d.name, 'path': str(d)} for d in dirs],
    }


def patient_id(body):
    pid = str(body.get('patient_id', '')).strip()
    if not PATIENT_ID_RE.match(pid):
        raise ApiError(HTTPStatus.BAD_REQUEST, f'ID de paciente no valido: {pid!r}')
    return pid


def api_session_start(body):
    pid = patient_id(body)
    folder = records_dir() / pid
    folder.mkdir(exist_ok=True)
    return {'dir': str(folder), 'stamp': datetime.datetime.now().strftime(STAMP_FMT)}


def api_session_notes(body):
    pid = patient_id(body)
    phase = str(body.get('phase', ''))
    stamp = str(body.get('stamp', ''))
    if phase not in PHASES or not STAMP_RE.match(stamp):
        raise ApiError(HTTPStatus.BAD_REQUEST, 'Fase o sello de tiempo no validos')
    folder = records_dir() / pid
    folder.mkdir(exist_ok=True)
    path = folder / f'{pid}_{phase}_{stamp}.txt'
    start = datetime.datetime.strptime(stamp, STAMP_FMT)
    content = (
        f'Paciente: {pid}\n'
        f'Fase: {phase.upper()}\n'
        f'Inicio grabacion: {start:%Y-%m-%d %H:%M:%S}\n'
        f'Guardado: {datetime.datetime.now():%Y-%m-%d %H:%M:%S}\n'
        f'\nObservaciones:\n{body.get("notes", "")}\n')
    try:
        # 'x': nunca sobrescribe una sesion anterior.
        with open(path, 'x', encoding='utf-8') as f:
            f.write(content)
    except FileExistsError:
        raise ApiError(HTTPStatus.CONFLICT, f'Ya existe {path.name}')
    return {'file': str(path)}


GET_ROUTES = {
    '/api/records/status': lambda _q: records_status(),
    '/api/fs/list': api_list,
}
POST_ROUTES = {
    '/api/records/create': api_create,
    '/api/records/select': api_select,
    '/api/session/start': api_session_start,
    '/api/session/notes': api_session_notes,
}


class Handler(SimpleHTTPRequestHandler):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(SCRIPT_DIR), **kwargs)

    def do_GET(self):
        url = urlparse(self.path)
        if url.path.startswith('/api/'):
            self.dispatch(GET_ROUTES, url.path, parse_qs(url.query))
        else:
            super().do_GET()

    def do_POST(self):
        url = urlparse(self.path)
        try:
            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length) or b'{}')
        except ValueError:
            self.send_json(HTTPStatus.BAD_REQUEST, {'error': 'JSON no valido'})
            return
        self.dispatch(POST_ROUTES, url.path, body)

    def dispatch(self, routes, path, arg):
        handler = routes.get(path)
        if handler is None:
            self.send_json(HTTPStatus.NOT_FOUND, {'error': f'No existe {path}'})
            return
        try:
            self.send_json(HTTPStatus.OK, handler(arg))
        except ApiError as e:
            self.send_json(e.status, {'error': str(e)})
        except OSError as e:
            self.send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {'error': str(e)})

    def send_json(self, status, obj):
        data = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--bind', default='0.0.0.0')
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    print(f'Monitor web en http://{args.bind}:{args.port}/ros2_gait_monitor.html')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
