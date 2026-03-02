from typing import Literal, Tuple, get_args

Color = Literal['grey', 'red', 'green', 'yellow', 'blue', 'purple', "cyan", 'white']
colorsList: Tuple[Color, ...] = get_args(Color)


def state(color: Color = 'red'):
    def h(f):
        def g(state, *args, **kwargs):
            state.indent(f"1;{30 + colorsList.index(color)}", f.__name__, args, kwargs)

            f(state, *args, **kwargs)

            state.unindent(f.__name__)

        return g
    return h

class State:
    def __init__(self) -> None:
        self.stack: list[str] = []
        self.instant = False
        self.printQueue = None

        self.modeA(3)

    def indent (self, c: str, name: str, args, kwargs: dict):
        self.print(f"\033[{c}m┌──{name}" + f"({", ".join([str(e) for e in args] + [f"{k}={v}" for k, v in kwargs.items()])})")
        self.stack.append(c)

    def unindent (self, name: str):
        c = self.stack.pop()
        self.print(f"\033[{c}m└──{name}")

    def print (self, *msgData):
        msg = " ".join(str(e) for e in msgData)

        s = "".join(f"\033[{c}m│  " for c in self.stack) + "\033[0m"

        for line in msg.splitlines():
            print(s + line)


    @state(color='red')
    def modeA(self, n: int):
        if n == 0:
            return

        self.modeA(n - 1)
        self.modeB("a", i=n, n=n)

    @state(color='green')
    def modeB(self, a, i: int, n):
        self.print("B\n", i)

State()