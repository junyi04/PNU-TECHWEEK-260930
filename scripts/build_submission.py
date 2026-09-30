"""Create a portable Webots project archive with no logs, venv, or test observers."""
from pathlib import Path
import hashlib
import json
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    output = ROOT/'dist'
    output.mkdir(exist_ok=True)
    files = [ROOT/'README.md', ROOT/'worlds/apartment.wbt', ROOT/'worlds/mission_demo.wbt',
             ROOT/'scripts/build_submission.py']
    for folder in ('controllers/apple_collector', 'protos'):
        files.extend(p for p in (ROOT/folder).rglob('*') if p.is_file() and
                     not any(part in ('logs', '__pycache__', '.git') for part in p.relative_to(ROOT).parts)
                     and p.suffix != '.pyc' and p.name != '.gitignore')
    manifest = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(files)}
    archive = output/'PNU-Robot-MVP.zip'
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as z:
        for p in sorted(files):
            z.write(p, 'PNU-Robot-MVP/'+p.relative_to(ROOT).as_posix())
        z.writestr('PNU-Robot-MVP/SHA256.json', json.dumps(manifest, indent=2))
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
    print(f'{archive} ({archive.stat().st_size:,} bytes, {len(files)} files)')


if __name__ == '__main__':
    main()
