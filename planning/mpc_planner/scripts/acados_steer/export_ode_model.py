from acados_template import *
from casadi import SX, vertcat, sin, cos, atan, tan

def export_ode_model() -> AcadosModel:
    model_name = "nmpc_planner_steer"

    # set up states & controls
    x = SX.sym("x")
    y = SX.sym("y")
    v = SX.sym("x_d")
    theta = SX.sym("theta")
    beta = SX.sym("beta")

    x = vertcat(x, y, v, theta, beta)

    A = SX.sym("A")
    D = SX.sym("D")
    u = vertcat(A, D)

    # xdot
    x_dot = SX.sym("x_dot")
    y_dot = SX.sym("y_dot")
    v_dot = SX.sym("v_dot")
    theta_dot = SX.sym("theta_dot")
    beta_dot = SX.sym("beta_ddot")

    xdot = vertcat(x_dot, y_dot, v_dot, theta_dot, beta_dot)

    # algebraic variables
    # z = None

    # parameters
    p = []
    lr = 1.4375
    # dynamics
    f_expl = vertcat(v * cos(theta + beta), v * sin(theta + beta), A, (v/lr)*sin(atan(beta)), 0.5*tan(D))

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