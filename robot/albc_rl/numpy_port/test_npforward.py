"""Check the numpy port against torch-produced golden vectors to atol=1e-5.

Run on the dev machine (numpy only, no torch needed):
    python3 test_npforward.py

Each assertion is a layer-by-layer parity check so any mismatch localizes the bug.
"""
import os
import numpy as np

import npforward as npf

HERE = os.path.dirname(os.path.abspath(__file__))
ATOL = 1e-5


def _load(name):
    return np.load(os.path.join(HERE, name))


def _check(label, got, want, atol=ATOL):
    got = np.asarray(got)
    want = np.asarray(want)
    ok = got.shape == want.shape and np.allclose(got, want, atol=atol)
    maxerr = np.abs(got - want).max() if got.shape == want.shape else float("nan")
    status = "PASS" if ok else "FAIL"
    print("  [{}] {:28s} max|err|={:.2e}  shape={}".format(status, label, maxerr, got.shape))
    assert ok, "{}: shape {} vs {}, max err {}".format(label, got.shape, want.shape, maxerr)


def test_teacher():
    print("== teacher (normalizer + actor MLP) ==")
    w = _load("weights_teacher.npz")
    g = _load("golden/golden_teacher.npz")
    actor = npf.TeacherActor(w)

    obs_n = actor.normalize(g["obs"])
    _check("obs_normalized", obs_n, g["obs_normalized"])

    action = actor.act(obs_n, g["latent"])
    _check("action", action, g["action"])


def test_tcn():
    print("== student TCN encoder ==")
    w = _load("weights_tcn.npz")
    g = _load("golden/golden_tcn.npz")
    enc = npf.StudentTCN(w)

    # submodule-by-submodule, reusing torch intermediates as inputs where useful
    win = g["input_window"]
    b, h, d = win.shape

    # channel_transform + ELU
    ct = npf.elu(npf.linear(win.reshape(b * h, d), enc.ct_w, enc.ct_b)).reshape(b, h, -1)
    _check("after_channel_transform", ct, g["after_channel_transform"])

    xt = np.transpose(ct, (0, 2, 1))
    _check("after_transpose", xt, g["after_transpose"])

    x = xt
    for cw, cb in enc.convs:
        x = npf.elu(npf.conv1d(x, cw, cb, stride=1))
    _check("after_conv", x, g["after_conv"])

    xf = x.reshape(b, -1)
    _check("after_flatten", xf, g["after_flatten"])

    xh = npf.linear(npf.elu(npf.linear(xf, enc.h0_w, enc.h0_b)), enc.h3_w, enc.h3_b)
    xh = npf.linear(
        npf.layer_norm(npf.elu(npf.linear(xf, enc.h0_w, enc.h0_b)), enc.ln_g, enc.ln_b),
        enc.h3_w, enc.h3_b,
    )
    _check("after_head", xh, g["after_head"])

    # full forward
    z = enc.forward(win)
    _check("latent (full forward)", z, g["latent"])


def test_gru():
    print("== student GRU encoder (stateful) ==")
    w = _load("weights_gru.npz")
    g = _load("golden/golden_gru.npz")
    enc = npf.StudentGRU(w)

    seq = g["input_seq"]            # (1, 4, 87)
    T = seq.shape[1]
    hidden = enc.init_hidden(1)
    _check("init_hidden", hidden, g["init_hidden"][0])  # golden (1,1,128) -> (1,128)

    latents = []
    for t in range(T):
        z, hidden = enc.step(seq[:, t, :], hidden)
        latents.append(z)
    latents = np.stack(latents, axis=0)   # (T, 1, 9)
    _check("latents_per_step", latents, g["latents_per_step"])

    # final hidden matches full-seq hidden
    _check("hidden_final", hidden[0], g["hidden_full_seq"][0, 0])


def _gru_golden_is_current():
    """True only if the GRU golden/weights match the current POLICY_OBS_DIM (69)."""
    from np_policy import POLICY_OBS_DIM
    try:
        g = _load("golden/golden_gru.npz")
        w = _load("weights_gru.npz")
    except IOError:
        return False
    return (g["input_seq"].shape[-1] == POLICY_OBS_DIM
            and w["gru.weight_ih_l0"].shape[1] == POLICY_OBS_DIM)


if __name__ == "__main__":
    test_teacher()
    test_tcn()
    if _gru_golden_is_current():
        test_gru()
    else:
        print("== student GRU encoder (stateful) ==")
        print("  [SKIP] no 69D GRU distill checkpoint/golden yet (TCN-only deploy)")
    print("\nALL PARITY CHECKS PASSED (atol={})".format(ATOL))
