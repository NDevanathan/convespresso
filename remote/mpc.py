import cvxpy as cp
from mhe import MHE


class TPTrackerMPC:
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
        self, A, B, c, c1, c2, target_p, target_T, H=None, enable_input_constraints=True
    ):
        # temperature dynamics
        self.A = A
        self.B = B
        self.c = c
        self.c1 = c1
        self.c2 = c2
        self.n, self.m = B.shape

        # output targets
        self.target_p = target_p
        self.target_T = target_T
        self.horizon = len(target_T)

        # MPC lookahead
        self.H = H if H is not None else self.horizon

        self.enable_input_constraints = enable_input_constraints

    def __call__(self, t, p, z):
        """Get MPC control at time index t, corresponding state"""
        H = (
            self.H
            if t + self.H < self.horizon
            else self.H - (t + self.H - self.horizon)
        )

        # state
        z_ = cp.Variable((H + 1, self.n))
        p_ = cp.Variable((H + 1))

        # control
        u1 = cp.Variable((H, self.m))
        u2 = cp.Variable((H))

        obj = 0.0
        cons = [
            z_[0] == z,
            p_[0] == p,
        ]
        if self.enable_input_constraints:
            cons.extend(
                [
                    u1 >= 0,
                    u1 <= 1,
                    u2 >= 0,
                    u2 <= 1,
                ]
            )
        for i, _t in enumerate(range(t, t + H)):
            obj += cp.square(z_[i, 0] - self.target_T[_t])
            obj += cp.square(p_[i] - self.target_p[_t])

            # dynamics constraints
            cons.append(z_[i + 1, :] == self.A @ z_[i, :] + self.B @ u1[i, :] + self.c)
            cons.append(p_[i + 1] == self.c1 * p_[i] + self.c2 * u2[i])

        prob = cp.Problem(cp.Minimize(obj), cons)
        prob.solve()

        return u1.value[0, :], u2.value[0]


if __name__ == "__main__":
    # test
    import numpy as np
    import matplotlib.pyplot as plt
    import pickle

    A, B, c = pickle.load(open("../notebooks/temp_dynamics.p", "rb"))
    n, m = B.shape

    # make something up
    c1 = np.random.rand()
    c2 = np.random.rand()

    N = 100
    target_p = np.array([0.1 * i for i in range(N)])
    target_T = np.array([34 + 0.5 * i for i in range(N)])

    controller = TPTrackerMPC(
        A, B, c, c1, c2, target_p, target_T, H=20, enable_input_constraints=False
    )

    p0 = 1.0
    z0 = np.ones(n) * 34.0

    # state estimation
    C = np.zeros((1, n))
    C[:,0] = 1.
    mhe = MHE(A, B, c, C, z0, 20)

    p = np.zeros(N)
    z = np.zeros((N, n))
    u1 = np.zeros((N, m))
    u2 = np.zeros(N)
    p[0] = p0
    z[0] = z0
    for i in range(N - 1):
        u1[i], u2[i] = controller(i, p[i], mhe(z[i][0]))
        mhe.update(u1[i])

        z[i + 1] = A @ z[i] + B @ u1[i] + c
        p[i + 1] = c1 * p[i] + c2 * u2[i]

    plt.plot(z[:, 0], "--", label="temp")
    plt.plot(target_T, label="target temp")
    plt.legend()
    plt.show()

    plt.plot(p, "--", label="pressure")
    plt.plot(target_p, label="target pressure")
    plt.legend()
    plt.show()

    plt.plot(u1, label="u1")
    plt.plot(u2, label="u2")
    plt.legend()
    plt.show()
