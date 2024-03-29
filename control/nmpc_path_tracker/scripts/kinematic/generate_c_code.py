from acados_template import *
import acados_template as at
from export_ode_model import *
import numpy as np
import scipy.linalg
from ctypes import *
from os.path import dirname, join, abspath

ACADOS_PATH = "/home/rohan/tools/acados"

# create render arguments
ocp = AcadosOcp()

# export model
model = export_ode_model()
ocp.model = model

Tf = 4.0
N = 40
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
ny_e = nx

# set ocp_nlp_dimensions
ocp.dims.nx  = nx
ocp.dims.ny  = ny
ocp.dims.ny_e = ny_e
ocp.dims.nbx = 0
ocp.dims.nbu = nu
ocp.dims.nbx_e = 0
ocp.dims.nu  = model.u.size()[0]
ocp.dims.N   = N

# set cost
Q_mat = np.diag([3.5, 3.5, 3.5, 0.0])  # [x,y,th, beta]
R_mat = np.diag([25]) # [delta]
Q_mat_e = np.diag([0, 0, 0, 0.0])

# set weighting matrices
ocp.cost.cost_type = "LINEAR_LS"
ocp.cost.cost_type_e = "LINEAR_LS"

ocp.cost.W_e = Q_mat_e
ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat) 

ocp.cost.Vx = np.zeros((ny, nx))
ocp.cost.Vx[:nx, :nx] = np.eye(nx)

Vu = np.zeros((ny, nu))
Vu[nx : nx + nu, 0:nu] = np.eye(nu)
ocp.cost.Vu = Vu

ocp.cost.Vx_e = np.eye(nx)
ocp.cost.yref = np.zeros((ny,))
ocp.cost.yref_e = np.zeros((ny_e,))

# set parameter
ocp.parameter_values = np.array([0.0])

# set constraints
ocp.constraints.lbu = np.array([-0.7])
ocp.constraints.ubu = np.array([+0.7])
ocp.constraints.idxbu = np.array([0])
ocp.constraints.x0 = np.array([0, 0, 0, 0])

## set QP solver
#ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_OSQP'
ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
ocp.solver_options.qp_solver_warm_start = 1
ocp.solver_options.qp_solver_iter_max = 100
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
ocp.solver_options.integrator_type = 'ERK'
ocp.solver_options.nlp_solver_max_iter = 200

# set prediction horizon
ocp.solver_options.tf = Tf
#ocp.solver_options.nlp_solver_type = 'SQP_RTI'
ocp.solver_options.nlp_solver_type = 'SQP'

# set header path
ocp.acados_include_path  = f'{ACADOS_PATH}/include'
ocp.acados_lib_path      = f'{ACADOS_PATH}/lib'

acados_ocp_solver = AcadosOcpSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )

print('>> NMPC exported')