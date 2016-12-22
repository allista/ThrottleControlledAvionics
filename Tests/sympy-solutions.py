from sympy.abc import t, g, k
from sympy import sqrt, Function, Derivative, dsolve

if __name__ == '__main__':
    x, y = map(Function, 'xy')
    dx = Derivative(x(t), t)
    dy = Derivative(y(t), t)
    eq1 = dx+k*x(t)/sqrt(x(t)+y(t))
    eq2 = dy+k*y(t)/sqrt(x(t)+y(t))
    s = dsolve((eq1, eq2))
    print(s)
