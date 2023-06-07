import cvxpy as cp


class MHE:
    """Moving horizon estimator

    x^\dot = Ax + Bu + c + w,
    y = Cx + v,

    w is process noise,
    v is measurement noise
    """

    def __init__(self, A, B, c, C, x0, horizon):
        self.A = A
        self.B = B
        self.c = c
        self.C = C
        self.horizon = horizon
        self.n, self.m = self.B.shape
        self.p = self.C.shape[0]

        # history of measurements and applied controls
        self.y_history = []
        self.u_history = []

        # previous horizon-1 state estimate
        self.x0 = x0.copy()

    def update(self, u):
        self.u_history.append(u)
        if len(self.u_history) > self.horizon:
            self.u_history.pop(0)

    def observe(self, y):
        """ Update history """
        self.y_history.append(y)
        if len(self.y_history) > self.horizon:
            self.y_history.pop(0)
        return len(self.y_history)

    def __call__(self, y):
        """State estimation using new observation y"""
        N = self.observe(y)

        x = cp.Variable((N, self.n))
        w = cp.Variable((N, self.n))
        # v = cp.Variable((N, self.p))

        cons = []
        for i in range(N):
            if i < N - 1:
                cons.append(
                    x[i + 1, :] == self.A @ x[i, :] + self.B @ self.u_history[i] + w[i, :]
                )
            # cons.append(self.y_history[i] == self.C @ x[i, :] + v[i, :])
            cons.append(self.y_history[i] == self.C @ x[i, :])

        # solve mhe problem
        # obj = cp.sum_squares(x[0, :] - self.x0) + cp.sum_squares(v) + cp.sum_squares(w)
        obj = cp.sum_squares(x[0, :] - self.x0) + cp.sum_squares(w)
        prob = cp.Problem(cp.Minimize(obj), cons)
        prob.solve()

        # update initial state estimate
        self.x0 = x.value[1, :] if N == self.u_history else x.value[0, :]

        # return state estimate
        return x.value[-1, :]


if __name__ == "__main__":
    # test
    import numpy as np

    n, m, p = 5, 3, 3
    T = 50
    H = 10

    A = np.random.randn(n, n)
    A /= np.max(np.abs(np.linalg.eigvals(A)))

    B = np.random.randn(n, m)
    C = np.random.randn(p, n)
    c = np.random.randn(n) / n

    x0 = np.random.randn(n)
    u = np.random.randn(T, m)

    estimator = MHE(A, B, c, C, x0, H)
    x_true = np.zeros((T, n))
    x_est = np.zeros((T, n))

    x_true[0, :] = x0
    x_est[0, :] = x0

    for t in range(T):
        y = C @ x_true[t, :] + 0.25 * np.random.randn(p)
        if t < T - 1:
            x_true[t + 1, :] = (
                A @ x_true[t, :] + B @ u[t, :] + c + 0.25 * np.random.randn(n)
            )

        x_est[t, :] = estimator(y)
        estimator.update(u[t, :])

    import matplotlib.pyplot as plt

    plt.plot(x_true[:, 0], label="true")
    plt.plot(x_est[:, 0], "--", label="est")
    plt.legend()
    plt.show()
