## Install
```
uv venv
source .venv/bin/activate
uv pip install -e ".[dev]"
uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu129
```

## Run
```
source .venv/bin/activate
python hiro/franka.py
```