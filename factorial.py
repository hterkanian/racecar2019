#!/usr/bin/env python

"""
Example code on use of assert statements
"""
from types import * 

def factorial(n):
    """Returns the factorial of n, a positive integer."""
    assert type(n) is IntType, 'Input is ' +\
            str(type(n)) +\
            ', input to factorial must be an integer.'
    assert n >= 0, 'Input must be a positive integer.'
    if n == 0:
        return 1
    else:
        return n * factorial(n-1)


if __name__ == '__main__':
    pass
