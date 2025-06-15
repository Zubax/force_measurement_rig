import click
import asyncio

from functools import wraps
from typing import Callable, Coroutine, Any


def inform(msg: str, fg: str | None = None, reset: bool = True, nl: bool = True) -> None:
    click.secho(msg, err=True, fg=fg, bold=True, nl=nl, reset=reset)


def coroutine(f: Callable[..., Coroutine[Any, Any, Any]]) -> Callable[..., Any]:
    @wraps(f)
    def wrapper(*args: Any, **kwargs: Any) -> Any:
        return asyncio.run(f(*args, **kwargs))

    return wrapper
