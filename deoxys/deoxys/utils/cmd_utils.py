BCOLORS = {"blue": "\033[94m", "red": "\033[91m", "end": "\033[0m"}


def color_print(color=None, *args):

    if color is None:
        print(*args)
    else:
        assert color in BCOLORS, "Color not defined!!!"
        print(BCOLORS[color], *args, BCOLORS["end"])


def WARNING_PRINT(*args):
    color_print("red", *args)
