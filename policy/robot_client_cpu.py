"""The STOCK lerobot robot client, with one compatibility shim applied first.

Nothing about the client is reimplemented: this imports `async_client` from
`lerobot.async_inference.robot_client` and calls it, so the CLI, the threads, the queue and
the aggregation are byte-for-byte the upstream ones. The only addition is the two lines below.

Why they are needed: the policy server runs pi0 on `cuda` and pickles action tensors that
still live in GPU memory, and this robot machine has a CPU-only torch. `pickle.loads` then
tries to restore the storages onto a CUDA device and dies with

    RuntimeError: Attempting to deserialize object on a CUDA device but
                  torch.cuda.is_available() is False

before `--client_device` is ever consulted. Remapping at load time fixes it here, from the
robot side, without needing anything changed or restarted on the DGX.

The real root cause is server-side: upstream `PolicyServer._predict_action_chunk` ends with
`action_tensor = action_tensor.detach().cpu()`, and `policy_server_persistent.py` dropped
that line. Once the DGX server carries it again these two lines become a no-op (mapping CPU
storages to CPU costs nothing), so it is safe to keep them either way.

The same shim is already used by `dummy_robot_client.py` for the same reason.
"""

import io

import torch

torch.storage._load_from_bytes = lambda b: torch.load(io.BytesIO(b), map_location="cpu", weights_only=False)

from lerobot.async_inference.robot_client import async_client  # noqa: E402 - must follow the shim
from lerobot.utils.import_utils import register_third_party_plugins  # noqa: E402

if __name__ == "__main__":
    register_third_party_plugins()
    async_client()
