"""Create clearly labelled SYNTHETIC input for end-to-end replay smoke tests.

This is not PICO data and must never be presented as a live-controller test.
"""
from dataclasses import asdict
import json
from pathlib import Path
import time
import numpy as np

from xr_heading import heading_basis
from xr_input import ControllerSample
from xr_recording import jsonable


def main():
    path = Path(__file__).resolve().parents[1] / "logs/xr/synthetic_replay_fixture.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    # A yawed and pitched HMD, while raw WORLD +Y remains gravity-aligned.
    q = np.array([.9164595, .0919527, .3874729, -.03887696])
    basis = heading_basis(q)
    origin = np.array([.1,1.,-.2])
    dt, t = .02, 0.
    with path.open("w", encoding="utf-8") as output:
        output.write(json.dumps({"type":"meta", "format":"rm65-xr-v1", "source":"synthetic-test", "config":{"frame":"robot"}})+"\n")
        for axis in range(3):
            for sign in [1., -1.]:
                for index in range(90):
                    active = 10 <= index < 80
                    distance = min(.02, max(0., (index-10)*dt*.025)) if active else 0.
                    pos = origin + basis[:,axis]*distance*sign
                    sample = ControllerSample(pos, np.array([1.,0.,0.,0.]), float(active), 0., False,
                                              time.time(), True, headset_quat_wxyz=q, headset_stamp=time.time())
                    payload = asdict(sample)
                    payload["age"], payload["head_age"] = .001, .001
                    output.write(json.dumps({"type":"frame", "t":round(t,4), "sample":payload},default=jsonable)+"\n")
                    t += dt
    print(path)


if __name__ == "__main__":
    main()
