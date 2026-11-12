"""GUI_android_tk_cfFF.py の結合テスト（偽ボード相手に GUI を動かす。画面と ROS 2 が必要で数分かかる）

CFFF_GUI_TEST=1 のときだけ動かす（実機に届かないよう ROS_DOMAIN_ID=87・ROS_LOCALHOST_ONLY=1 で起動する）:
  cd ~/ArmRobot/src/py_pkg && CFFF_GUI_TEST=1 python3 -m pytest -q -s test/test_gui_cfff_integration.py
"""
import os
import subprocess
import sys

import pytest


@pytest.mark.skipif(os.environ.get('CFFF_GUI_TEST') != '1' or not os.environ.get('DISPLAY'),
                    reason='CFFF_GUI_TEST=1 と DISPLAY が必要')
def test_gui_with_fake_robot(tmp_path):
    env = dict(os.environ, ROS_DOMAIN_ID='87', ROS_LOCALHOST_ONLY='1')
    harness = os.path.join(os.path.dirname(__file__), 'cfff_gui_harness.py')
    proc = subprocess.run([sys.executable, harness, str(tmp_path)], env=env, capture_output=True, text=True,
                          timeout=900)
    print(proc.stdout)
    assert proc.returncode == 0, proc.stdout[-4000:] + proc.stderr[-4000:]
    assert 'NG  ' not in proc.stdout
