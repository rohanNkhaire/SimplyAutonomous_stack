from acados_template import *
from casadi import SX, vertcat, sin, cos, atan, tan

def export_ode_model() -> AcadosModel:
    model_name = "nmpc_kinematic_controller"

    # set up states & controls
    x = SX.sym("x")
    y = SX.sym("y")
    theta = SX.sym("theta")
    beta = SX.sym("beta")

    x = vertcat(x, y, theta, beta)

    D = SX.sym("D")
    u = vertcat(D)

    # xdot
    x_dot = SX.sym("x_dot")
    y_dot = SX.sym("y_dot")
    theta_dot = SX.sym("theta_dot")
    beta_dot = SX.sym("beta_ddot")

    xdot = vertcat(x_dot, y_dot, theta_dot, beta_dot)

    # algebraic variables
    # z = None

    # parameters
    v = SX.sym("v")
    p = v

    # vehicle model
    lr = lf = 1.4375
    wb = lr + lf
    
    # dynamics
    f_expl = vertcat(v * cos(theta + beta), v * sin(theta + beta), (v/wb)*(tan(D)), atan(lr*tan(D))/(lr+lf))

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