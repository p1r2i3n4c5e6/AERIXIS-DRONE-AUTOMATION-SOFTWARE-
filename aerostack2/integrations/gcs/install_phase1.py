"""Install the reviewed GCS patch after filesystem approval; never restart the server."""
from pathlib import Path
import hashlib
import shutil
from datetime import datetime

here = Path(__file__).resolve().parent
target = Path('/home/prince/custom_gcs/server.py')
expected = (here / 'original_server.sha256').read_text().strip()
if hashlib.sha256(target.read_bytes()).hexdigest() != expected:
    raise SystemExit('GCS source changed since review; refusing to overwrite')
helper = target.with_name('bt_execution.py')
if helper.exists():
    raise SystemExit('bt_execution.py already exists; review before replacing it')
staged = Path('/tmp/as2_gcs_phase1/server.py')
compile(staged.read_text(), str(staged), 'exec')
backup = target.with_name('server.py.before_phase1_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
shutil.copy2(target, backup)
shutil.copy2(here / 'bt_execution.py', helper)
shutil.copyfile(staged, target)
print(f'Installed GCS execution integration. Backup: {backup}')
print('Existing server process was not restarted; changes apply at next launch.')
