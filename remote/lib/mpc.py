import cvxpy as cp
import numpy as np


class TempTrackerMPC:
    """
    Simply track provided temperature and pressure trajectories

    # Temperature dynamics
    z^\dot = Az + Bu_1 + c
    T = z_1

    # pressure dynamics
    \dot p = c_1 p + c_2 u_2

    # stage cost
    (p - p^tar)^2 + (T - T^tar)^2
    """

    def __init__(
        self, A, B, c, target_T, H=None, enable_input_constraints=True
    ):
        # temperature dynamics
        self.A = A
        self.B = B
        self.c = c
        self.n, self.m = B.shape

        # output targets
        self.target_T = target_T
        try:
            self.horizon = len(target_T)
        except:
            self.horizon = np.inf

        # MPC lookahead
        self.H = H if H is not None else self.horizon
        self.enable_input_constraints = enable_input_constraints

    def __call__(self, t, z):
        """Get MPC control at time index t, corresponding state"""
        H = (
            self.H
            if t + self.H < self.horizon
            else self.H - (t + self.H - self.horizon)
        )

        # state
        z_ = cp.Variable((H + 1, self.n))

        # control
        u1 = cp.Variable((H, self.m))

        obj = cp.sum_squares(u1) / self.m
        cons = [z_[0] == z]
        if self.enable_input_constraints:
            cons.extend([u1 >= 0, u1 <= 1])
        for i, _t in enumerate(range(t, t + H)):
            obj += cp.square(z_[i, 0] - (self.target_T[_t] if np.isfinite(self.horizon) else self.target_T))
            cons.append(z_[i + 1, :] == self.A @ z_[i, :] + self.B @ u1[i, :] + self.c)

        prob = cp.Problem(cp.Minimize(obj), cons)
        prob.solve()

        return u1.value[0, :]


if __name__ == "__main__":
    # test
    import numpy as np
    import matplotlib.pyplot as plt
    import pickle
    from tqdm import tqdm

    N = 100
    step_size = 0.5

    A, B, c = pickle.load(open("notebooks/temp_dynamics.p", "rb"))
    n, m = B.shape

    A = np.eye(n) + step_size * A
    B *= step_size
    c *= step_size

    # target_T = np.array([34 + 0.1 * i for i in range(N)])
    target_T = 92

    controller = TempTrackerMPC(
        A, B, c, target_T, H=50, enable_input_constraints=True
    )

    z0 = np.ones(n) * 34.0

    # state estimation
    from mhe import MHE
    C = np.zeros((1, n))
    C[:,0] = 1.
    mhe = MHE(A, B, c, C, z0, 20)

    p = np.zeros(N)
    z = np.zeros((N, n))
    u1 = np.zeros((N, m))
    z[0] = z0
    for i in tqdm(range(N - 1)):
        u1[i] = controller(i, mhe(z[i][0]))
        mhe.update(u1[i])
        z[i + 1] = A @ z[i] + B @ u1[i] + c

    plt.plot(step_size*np.arange(N), z[:, 0], "--", label="temp")
    # plt.plot(step_size*np.arange(N), target_T, label="target temp")
    plt.plot(step_size*np.arange(N), target_T * np.ones(N), label="target temp")
    plt.legend()
    plt.show()

    plt.plot(step_size*np.arange(N), u1, label="u1")
    plt.legend()
    plt.show()
