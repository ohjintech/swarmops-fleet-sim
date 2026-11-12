from __future__ import annotations

import asyncio
import logging

import uvicorn

from .chaos import create_app
from .config import Config
from .fleet import Fleet


async def _amain() -> None:
    cfg = Config()
    fleet = Fleet(cfg)
    server = uvicorn.Server(
        uvicorn.Config(create_app(fleet), host=cfg.chaos_host, port=cfg.chaos_port, log_level="warning")
    )
    async with asyncio.TaskGroup() as tg:
        tg.create_task(fleet.run())
        tg.create_task(server.serve())


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    try:
        asyncio.run(_amain())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
