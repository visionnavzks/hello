import numpy as np
import scipy.signal as signal
from scipy.optimize import minimize

class DoublePendulumCart:
    """
    Cart-Double-Pole Inverted Pendulum dynamics and MPC Controller.
    """
    def __init__(self, m_c=1.0, m_1=0.5, m_2=0.5, l_1=0.5, l_2=0.5, g=9.81):
        self.m_c = m_c
        self.m_1 = m_1
        self.m_2 = m_2
        self.l_1 = l_1
        self.l_2 = l_2
        self.g = g

        # Current state: [x, theta1, theta2, x_dot, theta1_dot, theta2_dot]
        # Angles are defined from the upright position (0 = upright)
        self.state = np.array([0.0, 0.05, -0.05, 0.0, 0.0, 0.0]) # Small initial angle
        self.dt = 0.02 # 50Hz

        # MPC variables
        self.setup_mpc()

    def get_M(self, theta1, theta2):
        """Mass matrix"""
        M = np.zeros((3, 3))
        M[0, 0] = self.m_c + self.m_1 + self.m_2
        M[0, 1] = (self.m_1 + self.m_2) * self.l_1 * np.cos(theta1)
        M[0, 2] = self.m_2 * self.l_2 * np.cos(theta2)

        M[1, 0] = M[0, 1]
        M[1, 1] = (self.m_1 + self.m_2) * self.l_1**2
        M[1, 2] = self.m_2 * self.l_1 * self.l_2 * np.cos(theta1 - theta2)

        M[2, 0] = M[0, 2]
        M[2, 1] = M[1, 2]
        M[2, 2] = self.m_2 * self.l_2**2
        return M

    def get_C_and_G(self, theta1, theta2, theta1_dot, theta2_dot):
        """Coriolis/Centrifugal and Gravity terms"""
        C = np.zeros(3)
        C[0] = -(self.m_1 + self.m_2) * self.l_1 * theta1_dot**2 * np.sin(theta1) - self.m_2 * self.l_2 * theta2_dot**2 * np.sin(theta2)
        C[1] = self.m_2 * self.l_1 * self.l_2 * theta2_dot**2 * np.sin(theta1 - theta2)
        C[2] = -self.m_2 * self.l_1 * self.l_2 * theta1_dot**2 * np.sin(theta1 - theta2)

        G = np.zeros(3)
        G[0] = 0.0
        G[1] = -(self.m_1 + self.m_2) * self.g * self.l_1 * np.sin(theta1)
        G[2] = -self.m_2 * self.g * self.l_2 * np.sin(theta2)

        return C + G

    def dynamics(self, t, state, u):
        """Nonlinear dynamics dx/dt = f(x, u)"""
        x, th1, th2, dx, dth1, dth2 = state

        M = self.get_M(th1, th2)
        CG = self.get_C_and_G(th1, th2, dth1, dth2)

        # B matrix mapping force to coordinates [x, th1, th2]
        B_sys = np.array([1.0, 0.0, 0.0])

        rhs = B_sys * u - CG
        accel = np.linalg.solve(M, rhs)

        return np.array([dx, dth1, dth2, accel[0], accel[1], accel[2]])

    def rk4_step(self, u):
        """Runge-Kutta 4th order integration"""
        k1 = self.dynamics(0, self.state, u)
        k2 = self.dynamics(0, self.state + 0.5 * self.dt * k1, u)
        k3 = self.dynamics(0, self.state + 0.5 * self.dt * k2, u)
        k4 = self.dynamics(0, self.state + self.dt * k3, u)

        self.state = self.state + (self.dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        # Keep angles reasonably bounded in [-pi, pi] for display purposes
        # But for MPC we want them near 0
        self.state[1] = (self.state[1] + np.pi) % (2*np.pi) - np.pi
        self.state[2] = (self.state[2] + np.pi) % (2*np.pi) - np.pi

    def setup_mpc(self):
        """Linearize system at upright equilibrium and setup MPC matrices"""
        # Linearization around x=0, th1=0, th2=0, dx=0, dth1=0, dth2=0
        # M_0 * q_ddot + G_0 * q = B * u
        # q_ddot = -M_0_inv * G_0 * q + M_0_inv * B * u

        M_0 = self.get_M(0.0, 0.0)

        # Partial derivative of G w.r.t q (at origin)
        # G1 = -(m1+m2)*g*l1*sin(th1) => dG1/dth1 = -(m1+m2)*g*l1
        # G2 = -m2*g*l2*sin(th2) => dG2/dth2 = -m2*g*l2
        dG_dq = np.zeros((3, 3))
        dG_dq[1, 1] = -(self.m_1 + self.m_2) * self.g * self.l_1
        dG_dq[2, 2] = -self.m_2 * self.g * self.l_2

        B_0 = np.array([[1.0], [0.0], [0.0]])

        M_0_inv = np.linalg.inv(M_0)

        A21 = -M_0_inv @ dG_dq
        B2 = M_0_inv @ B_0

        Ac = np.zeros((6, 6))
        Ac[0:3, 3:6] = np.eye(3)
        Ac[3:6, 0:3] = A21

        Bc = np.zeros((6, 1))
        Bc[3:6, :] = B2

        # Discretize
        sys_c = signal.StateSpace(Ac, Bc, np.eye(6), np.zeros((6, 1)))
        # Using StateSpace formulation instead of lti to prevent BadCoefficients warnings
        sys_d = sys_c.to_discrete(self.dt)
        self.Ad = sys_d.A
        self.Bd = sys_d.B

        # MPC parameters
        self.N = 30 # Horizon

        # Weights
        # States: x, th1, th2, dx, dth1, dth2
        self.Q = np.diag([10.0, 200.0, 200.0, 1.0, 10.0, 10.0])
        self.R = np.array([[0.1]])

        # Dense QP formulation: X = Phi * x0 + Gamma * U
        # J = X^T * Q_bar * X + U^T * R_bar * U
        #   = (Phi * x0 + Gamma * U)^T * Q_bar * (Phi * x0 + Gamma * U) + U^T * R_bar * U
        #   = U^T * (Gamma^T * Q_bar * Gamma + R_bar) * U + 2 * x0^T * Phi^T * Q_bar * Gamma * U + const

        n_x = 6
        n_u = 1

        self.Q_bar = np.kron(np.eye(self.N), self.Q)
        self.R_bar = np.kron(np.eye(self.N), self.R)

        self.Phi = np.zeros((self.N * n_x, n_x))
        self.Gamma = np.zeros((self.N * n_x, self.N * n_u))

        curr_A = np.eye(n_x)
        for i in range(self.N):
            curr_A = curr_A @ self.Ad
            self.Phi[i*n_x : (i+1)*n_x, :] = curr_A

            curr_B = self.Bd
            for j in range(i, -1, -1):
                self.Gamma[i*n_x : (i+1)*n_x, j*n_u : (j+1)*n_u] = curr_B
                curr_B = self.Ad @ curr_B

        self.H = self.Gamma.T @ self.Q_bar @ self.Gamma + self.R_bar
        self.H = (self.H + self.H.T) / 2 # Symmetrize to avoid numerical issues

        self.Phi_T_Q_Gamma = self.Phi.T @ self.Q_bar @ self.Gamma

        # MPC Limits
        self.u_max = 50.0

    def get_mpc_action(self):
        """Solve MPC to get control action"""
        # Linear term f = 2 * (x0^T * Phi^T * Q_bar * Gamma)
        # scipy minimize formulation: 0.5 * x^T * H * x + f^T * x
        # Our J = u^T * H * u + (2 * x0^T * Phi_T_Q_Gamma) * u
        # So we can pass 2*H to scipy if we wanted, but let's just write the cost func directly

        # If angle is too far from upright, MPC might fail/give garbage.
        if abs(self.state[1]) > np.pi/2 or abs(self.state[2]) > np.pi/2:
            return 0.0 # Just fall if it's too far (or could implement a swing up)

        x0 = self.state.copy()

        f = (x0.T @ self.Phi_T_Q_Gamma).T # shape (N*nu, )

        def cost(u):
            return u.T @ self.H @ u + 2 * f.T @ u

        def grad(u):
            return 2 * self.H @ u + 2 * f

        bounds = [(-self.u_max, self.u_max)] * self.N
        u0 = np.zeros(self.N)

        res = minimize(cost, u0, method='L-BFGS-B', jac=grad, bounds=bounds, options={'maxiter': 20, 'ftol': 1e-3})

        if res.success or res.status == 1: # Success or max iter
            return res.x[0]
        else:
            return 0.0
