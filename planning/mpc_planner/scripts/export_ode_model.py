from acados_template import *
from casadi import SX, vertcat, sin, cos

def export_ode_model() -> AcadosModel:
    model_name = "nmpc_planner"

    # set up states & controls
    x = SX.sym("x")
    y = SX.sym("y")
    v = SX.sym("x_d")
    theta = SX.sym("theta")

    x = vertcat(x, y, v, theta)

    A = SX.sym("A")
    YR = SX.sym("YR")
    u = vertcat(A, YR)

    # xdot
    x_dot = SX.sym("x_dot")
    y_dot = SX.sym("y_dot")
    v_dot = SX.sym("v_dot")
    theta_dot = SX.sym("theta_dot")

    xdot = vertcat(x_dot, y_dot, v_dot, theta_dot)

    # algebraic variables
    # z = None

    # parameters
    p_x = SX.sym("p_x")
    p_y = SX.sym("p_y")
    p_v = SX.sym("p_v")
    p_th = SX.sym("p_th")
    p_ex = SX.sym("p_ex")
    p_ey = SX.sym("p_ey")
    p = vertcat(p_x, p_y, p_v, p_th, p_ex, p_ey)

    # dynamics
    f_expl = vertcat(v * cos(theta), v * sin(theta), A, YR)

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    # model.z = z
    model.p = p
    model.name = model_name

    return model