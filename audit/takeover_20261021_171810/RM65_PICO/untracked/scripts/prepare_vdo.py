"""Install a small, idempotent native controller hook after verifying source text."""
import argparse
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def prepare(path):
    target = Path(path).resolve()
    hold_followup=ROOT/'patches/vdo-hold-ack.patch'
    hold_installed=subprocess.run(['git','-C',str(target),'apply','--reverse','--check',str(hold_followup)],capture_output=True).returncode==0
    followup = ROOT / 'patches/vdo-gripper-ack.patch'
    followup_installed = subprocess.run(['git','-C',str(target),'apply','--reverse','--check',str(followup)],capture_output=True).returncode==0
    if hold_installed:
        followup_installed = followup_installed or subprocess.run(['git','-C',str(target),'apply','--reverse','--check','--exclude=real/json_link.py',str(followup)],capture_output=True).returncode==0
    for name in ("vdo-controller-hook.patch", "vdo-windows-cli.patch", "vdo-exit-status.patch", "vdo-windows-guard.patch", "vdo-joint-bridge.patch", "vdo-gripper-ack.patch", "vdo-hold-ack.patch"):
        patch = ROOT / "patches" / name
        def git(*args):
            return subprocess.run(["git", "-C", str(target), "apply", *args, str(patch)],
                                  capture_output=True, text=True)
        if git("--reverse", "--check").returncode == 0:
            print(f"VDO patch already installed: {name}")
            continue
        if (name=='vdo-gripper-ack.patch' and hold_installed and followup_installed):
            print(f'VDO patch installed with hold follow-up: {name}')
            continue
        # The follow-up edits two bridge hunks. Verify its reverse plus the
        # remaining bridge files before treating the older patch as installed.
        if (name=='vdo-joint-bridge.patch' and followup_installed
                and git('--reverse','--check','--exclude=real/json_link.py','--exclude=real/real_arm.py').returncode==0):
            print(f'VDO patch installed with gripper follow-up: {name}')
            continue
        result = git("--check")
        if result.returncode:
            raise RuntimeError("Native source differs; patch not applied:\n" + result.stderr)
        result = git()
        if result.returncode:
            raise RuntimeError(result.stderr)
        print(f"VDO patch installed: {name}")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("path", nargs="?", default=str(ROOT.parent / "vdo-rm65b"))
    prepare(p.parse_args().path)
