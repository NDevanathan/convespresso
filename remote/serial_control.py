import datetime as dt
import time
from abc import ABCMeta, abstractmethod
from multiprocessing import Event, Pipe, Process, Queue

import cvxpy as cp
import numpy as np
import pause

from lib.mhe import MHE
from lib.mpc import TempTrackerMPC
from lib.serial_courier import SerialCourier

FREQ = 60  # Hz
PERIOD = 1 / FREQ
DELTA = dt.timedelta(seconds=PERIOD)  # seconds
PERIODS_PER_FRAME = 15

PRE_INF_DUR = 10
RAMP_DUR = 5
PRE_INF_LEVEL = 1 / 5
RAMP_LEVEL = 3 / 5
FLOW_TARG = 2


class CommProcess(Process):
    def __init__(self, act_pipe, targ_pipe, state_queue, brew_event, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.act_pipe = act_pipe
        self.targ_pipe = targ_pipe
        self.state_queue = state_queue
        self.brew_event = brew_event
        self.comms = SerialCourier()
        self.action = [0.0, 0.0]
        self.targets = [95.0, 9.0, 0.0]

    def run(self):
        i = 0
        start = dt.datetime.now()
        next = start
        brew_time = 0

        while True:
            self.state_queue.put(self.comms.get_state())
            while self.targ_pipe.poll():
                self.targets = self.targ_pipe.recv()
            while self.act_pipe.poll():
                self.action = self.act_pipe.recv()

            if not self.brew_event.is_set():
                start = next
                self.action[1] = 0.0
                self.comms.close_valve()
            else:
                brew_time = (next - start).total_seconds()

            self.comms.take_action(self.action[0], self.action[1])
            if i % PERIODS_PER_FRAME == 0:
                self.comms.refresh_display(
                    "ESPRESSO",
                    self.targets[0],
                    self.targets[1],
                    self.targets[2],
                    brew_time,
                )

            i += 1
            next += DELTA
            pause.until(next)


class LowPassFilter:
    def __init__(self, alpha_per_second):
        self.alpha_per_second = alpha_per_second
        self.state = None

    def apply(self, dt, value):
        if self.state is None:
            self.state = value
            return value
        else:
            alpha = 1 - np.exp(-self.alpha_per_second * dt)
            self.state = alpha * value + (1 - alpha) * self.state
            return self.state


class Controller(Process, metaclass=ABCMeta):
    def __init__(self, act_pipe, targ_pipe, state_queue, write_event, brew_event, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.act_pipe = act_pipe
        self.targ_pipe = targ_pipe
        self.state_queue = state_queue
        self.brew_event = brew_event
        self.write_event = write_event
        self.state = np.zeros(4)

        self.targets = [94.5, 9.0, 0.0]
        self.flow_coefs = [6.4634e02, -7.0024e01, 4.6624e00, -1.9119e-01]

    @abstractmethod
    def run(self):
        pass


class OnOff(Controller):
    def __init__(self, filter, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.filter = filter
        self.data = np.zeros((0, 5))

    def temp_control(self, secs):
        if self.state[0] < self.targets[0]:
            return 1.0
        else:
            return 0.0

    def flow_control(self, secs):
        if self.state[1] < self.targets[1]:
            return 1.0
        else:
            return 0.0001

    def run(self):
        start = dt.datetime.now()
        next = start
        start_secs = time.time()
        while True:
            while not self.state_queue.empty():
                obs = np.array(self.state_queue.get())
                obs[:2] = self.filter.apply(dt=PERIOD, value=obs[:2])
                self.state = obs
                t = time.time() - start_secs
                self.data = np.vstack((self.data, np.concatenate(([t], obs))))

            action = [0.0, 0.0]
            action[0] = self.temp_control((next - start).total_seconds())

            if self.brew_event.is_set():
                action[1] = self.flow_control((next - start).total_seconds())
            else:
                self.targets[2] = 0
                start = next

            if self.write_event.is_set():
                # do writing here
                np.savetxt(f'../logs/log_io_{int(time.time())}.csv', self.data, delimiter=',')
                print("Done writing")
                self.act_pipe.send([0.,0.])
                return

            self.act_pipe.send(action)
            self.targ_pipe.send(self.targets)
            next += DELTA
            pause.until(next)


class PID(Controller):
    def __init__(self, filter, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.filter = filter
        self.heat_last_error = None
        self.heat_last_integral = 0.0
        self.pump_last_error = None
        self.pump_last_integral = 0.0
        self.data = np.zeros((0, 5))

    def temp_control(self, secs):
        target = self.targets[0]
        curr_temp = self.state[0]

        alpha = 1 / 10
        beta = 1 / 30
        gamma = 0

        error = target - curr_temp
        proportional = error
        derivative = 0.0
        integral = self.heat_last_integral

        if not (self.heat_last_error is None):
            derivative = (error - self.heat_last_error) / PERIOD
            integral += self.heat_last_error * PERIOD

        self.heat_last_error = error
        return alpha * proportional + beta * integral + gamma * derivative

    def flow_control(self, secs):
        target = self.targets[1]
        curr_temp = self.state[1]

        alpha = 1 / 5
        beta = 0
        gamma = 0

        error = target - curr_temp

        proportional = error
        derivative = 0.0
        integral = self.pump_last_integral

        if not (self.pump_last_error is None):
            derivative = (error - self.pump_last_error) / PERIOD
            integral += self.pump_last_error * PERIOD

        self.pump_last_error = error
        return max(alpha * proportional + beta * integral + gamma * derivative, 0.0001)

    def run(self):
        start = dt.datetime.now()
        next = start
        start_secs = time.time()
        while True:
            while not self.state_queue.empty():
                obs = np.array(self.state_queue.get())
                obs[:2] = self.filter.apply(dt=PERIOD, value=obs[:2])
                self.state = obs
                t = time.time() - start_secs
                self.data = np.vstack((self.data, np.concatenate(([t], obs))))

            action = [0.0, 0.0]
            action[0] = self.temp_control((next - start).total_seconds())

            if self.brew_event.is_set():
                action[1] = self.flow_control((next - start).total_seconds())
            else:
                self.targets[2] = 0
                start = next

            if self.write_event.is_set():
                # do writing here
                np.savetxt(f'../logs/log_pid_{int(time.time())}.csv', self.data, delimiter=',')
                print("Done writing")
                self.act_pipe.send([0.,0.])
                return

            self.act_pipe.send(action)
            self.targ_pipe.send(self.targets)
            next += DELTA
            pause.until(next)


class IndependentMIAC(Controller):
    def __init__(self, filter, P0, A0, B0, c0, *args, num_states=1, **kwargs):
        super().__init__(*args, **kwargs)
        assert P0.shape[0] == num_states * 8 + 2
        self.num_states = num_states
        self.P = P0
        self.ahat = np.hstack((A0, B0, c0.reshape((-1, 1)))).flatten()
        self.Ahat = np.vstack((A0, np.eye(2*(num_states-1), 2*num_states)))
        self.Bhat = np.vstack((B0, np.eye(2*(num_states-1), 2*num_states)))
        self.chat = np.pad(c0, (0, 2*self.num_states-2), mode='constant')

        self.filter = filter
        self.targets = np.array(self.targets)
        self.state = None
        self.control = None
        self.last_state = np.zeros((0, 2))
        self.last_control = np.zeros((0, 2))

    def update_model(self):
        if self.last_state.shape[0] == self.num_states:
            last_obs = np.concatenate(
                (self.last_state.flatten(), self.last_control.flatten(), [1])
            )
            phi = np.kron(last_obs, np.eye(2))
            deriv = FREQ * (self.state - self.last_state[-1])
            # K = np.linalg.solve(np.eye(2) + phi @ self.P @ phi.T, phi @ self.P.T).T
            # self.ahat += K @ (deriv - phi @ self.ahat)
            # self.P = (np.eye(self.num_states * 8 + 2) - K @ phi) @ self.P
            self.ahat += PERIOD * self.P @ phi.T @ (deriv - phi) @ self.ahat
            self.P += -PERIOD * self.P @ phi.T @ phi @ self.P
        stacked = self.ahat.reshape((2, self.num_states * 4 + 1))
        self.Ahat = np.vstack(
            (
                stacked[:, : 2 * self.num_states],
                np.eye(2 * (self.num_states - 1), 2 * self.num_states),
            )
        )
        self.Bhat = np.vstack(
            (
                stacked[:, 2 * self.num_states : 4 * self.num_states],
                np.eye(2 * (self.num_states - 1), 2 * self.num_states),
            )
        )
        self.chat = np.pad(stacked[:, 4 * self.num_states], (0, 2*self.num_states - 2), mode='constant')
        print(self.Ahat)
        print(self.Bhat)
        print(self.chat)

    def compute_action(self):
        if self.state is None or self.last_state.shape[0] < self.num_states:
            return np.zeros(2)

        num_samples = 1 * FREQ
        r_cvx = cp.Variable((num_samples - 1, 2 * self.num_states))
        traj = cp.Variable((num_samples, 2 * self.num_states))

        traj_diff = traj[:, :2] - self.targets[None, :2]
        if not self.brew_event.is_set():
            traj_err = traj_diff @ np.diag([1, 0])
        else:
            traj_err = traj_diff @ np.diag([1, 1])
        obj = cp.sum_squares(traj_err) + cp.sum_squares(r_cvx)
        obj += 9 * cp.sum_squares(traj_err[-1])

        endo = traj[:-1] @ self.Ahat.T + self.chat[None]
        exo = r_cvx @ self.Bhat.T
        constraints = [
            traj[1:] == traj[:-1] + PERIOD * (endo + exo),
            0 <= r_cvx,
            r_cvx <= 1,
            r_cvx[0] == np.hstack((self.control, self.last_control[:-1].flatten())),
            traj[0] == np.hstack((self.state, self.last_state[:-1].flatten())),
        ]
        if self.num_states > 1:
            constraints += [
                r_cvx[:-1, :-2] == r_cvx[1:, 2:]
            ]
        if not self.brew_event.is_set():
            constraints += [r_cvx[1:, 1] == 0]

        prob = cp.Problem(cp.Minimize(obj), constraints)
        val = prob.solve()
        # print(val)

        # print(r_cvx.value[0])
        # print(traj.value[-1])
        # print("\n" + "*" * 20)
        return np.clip(r_cvx.value[1], 0, 1)

    def run(self):
        start = dt.datetime.now()
        next = start
        while True:
            while not self.state_queue.empty():
                obs = np.array(self.state_queue.get())
                obs[:2] = self.filter.apply(dt=PERIOD, value=obs[:2])
                print(obs)
                if not (self.state is None):
                    self.last_state, self.state = (
                        np.vstack((self.last_state, self.state)),
                        obs[:2],
                    )
                    self.last_control, self.control = (
                        np.vstack((self.last_control, self.control)),
                        obs[2:],
                    )
                else:
                    self.state = obs[:2]
                    self.control = obs[2:]
                if self.last_state.shape[0] > self.num_states:
                    self.last_state = self.last_state[-self.num_states :]
                if self.last_control.shape[0] > self.num_states:
                    self.last_control = self.last_control[-self.num_states :]
            print(self.last_state)
            print(self.last_control)

            # Update the model dynamics
            self.update_model()

            # Compute an optimal action
            action = list(self.compute_action())

            if not self.brew_event.is_set():
                start = next

            self.act_pipe.send(action)
            self.targ_pipe.send(list(self.targets))
            next += DELTA
            pause.until(next)
            print(dt.datetime.now())


class IndependentMRAC(Controller):
    def __init__(self, Am, Bm, cm, filter, kx, kr, gamma, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # We assume Am, Bm, cm, kx, kr, gamma are numpy arrays.
        # Each contains a hyperparameter for temperature first and
        # for pressure second.
        self.Am = Am
        self.Bm = Bm
        self.cm = cm

        self.kx = kx
        self.kr = kr
        self.gamma = gamma

        self.filter = filter
        self.targets = np.array(self.targets[:2])
        self.state = np.zeros(2)
        self.controls = np.zeros(2)
        self.r = np.array([], dtype=np.dtype("float64"))
        self.rt = None

    def update_model(self):
        # Implement MRAC model update method here
        err = self.state - self.targets
        self.kx += (
            -self.gamma * err * self.state * PERIOD * (1 - (self.controls - 0.5) ** 2)
        )
        self.kr += (
            -self.gamma * err * self.rt * PERIOD * (1 - (self.controls - 0.5) ** 2)
        )
        self.controls = np.clip(self.kx * self.state + self.kr * self.rt, 0, 1)
        print(self.controls)

    def compute_reference(self):
        if len(self.r) == 0:
            num_samples = 60 * FREQ
            r_cvx = cp.Variable((num_samples - 1, 2))
            traj = cp.Variable((num_samples, 2))

            traj_diff = traj - self.targets[None]
            traj_err = traj_diff @ np.diag([1, 1])
            obj = cp.sum_squares(traj_err) + cp.sum_squares(r_cvx)

            endo = traj[:-1] @ np.diag(self.Am) + self.cm[None]
            exo = r_cvx @ np.diag(self.Bm)
            constraints = [
                traj[1:] == traj[:-1] + PERIOD * (endo + exo),
                0 <= r_cvx,
                r_cvx <= 1,
                traj[0] == self.state,
            ]

            prob = cp.Problem(cp.Minimize(obj), constraints)
            prob.solve()

            self.r = r_cvx.value
            self.rt, self.r = self.r[0], self.r[1:]
        self.rt, self.r = self.r[0], self.r[1:]
        if not self.brew_event.is_set():
            self.rt, self.r = self.r[0], self.r[1:]
            self.r[1] = 0.0

    def run(self):
        start = dt.datetime.now()
        next = start
        while True:
            while not self.state_queue.empty():
                obs = np.array(self.state_queue.get())[:2]
                self.state = self.filter.apply(dt=PERIOD, value=obs)

            # Compute a reference signal if necessary
            if self.rt is None:
                self.compute_reference()

            # Update MRAC model parameters if necessary
            self.update_model()

            action = [0.0, 0.0]
            action[0] = self.controls[0]

            if self.brew_event.is_set():
                action[1] = self.controls[1]
            else:
                start = next

            self.act_pipe.send(action)
            self.targ_pipe.send(list(self.targets) + [0])
            next += DELTA
            pause.until(next)


class MPC(Controller):
    def __init__(
        self,
        act_pipe,
        targ_pipe,
        state_queue,
        write_event,
        brew_event,
        A,
        B,
        c,
        target_T,
        C,
        mhe_horizon,
        filter,
        *args,
        H=10,
        Ap=np.array([[-0.07032745]]),
        Bp=np.array([[0.95982973]]),
        cp=np.zeros(1),
        Cp=np.ones((1, 1)),
        target_P=9,
        **kwargs
    ):
        super().__init__(act_pipe, targ_pipe, state_queue, write_event, brew_event, *args, **kwargs)
        self.A = A
        self.B = B
        self.c = c
        self.C = C
        self.target_T = target_T
        self.mhe_horizon = mhe_horizon
        self.n, self.m = self.B.shape
        self.controller = TempTrackerMPC(A, B, c, target_T, H=H)
        self.controllerp = TempTrackerMPC(Ap, Bp, cp, target_P, H=H)
        self.filter = filter
        self.mhe = None
        self.mhep = None
        self.Ap = Ap
        self.Bp = Bp
        self.cp = cp
        self.Cp = Cp
        self.data = np.zeros((0, 5))

    def run(self):
        start = dt.datetime.now()
        next = start
        self.t = 0
        start_secs = time.time()
        while True:
            while not self.state_queue.empty():
                obs = np.array(self.state_queue.get())
                obs[:2] = self.filter.apply(dt=PERIOD, value=obs[:2])
                self.state = obs[:2]
                self.curr_u = obs[2:]
                tiem = time.time() - start_secs
                self.data = np.vstack((self.data, np.concatenate(([tiem], obs))))

            if self.state[0] >= 1e-5 and self.mhe is None:
                x0 = self.state[0] * np.ones(self.n)
                print(f'x0 = {x0}')
                self.mhe = MHE(self.A, self.B, self.c, self.C, x0, self.mhe_horizon)
                self.mhep = MHE(self.Ap, self.Bp, self.cp, self.Cp, 0.005, self.mhe_horizon)

            if not self.mhe is None:
                u = self.controller(self.t, self.mhe(self.state[0]))
                self.mhe.update(u)
            else:
                u = 0.

            action = [0.0, 0.0]
            action[0] = float(u)
            if self.brew_event.is_set():
                up = self.controllerp(self.t, self.mhep(self.state[1]))
                self.mhep.update(up)
                action[1] = max(float(up), 0.001)
            else:
                action[1] = 0.
                start = next

            if self.write_event.is_set():
                # do writing here
                np.savetxt(f'../logs/log_mpc_{int(time.time())}.csv', self.data, delimiter=',')
                print("Done writing")
                self.act_pipe.send([0.,0.])
                return

            self.act_pipe.send(action)
            self.targ_pipe.send(list(self.targets) + [0])
            next += DELTA
            self.t += 1
            pause.until(next)


if __name__ == "__main__":
    import pickle
    A, B, c = pickle.load(open("../notebooks/temp_dynamics.p", "rb"))
    A = np.eye(5) + PERIOD * A
    B *= PERIOD
    c *= PERIOD

    target_T = 90

    mhe_horizon = 10
    C = np.zeros((1, len(c)))
    C[:, 0] = 1.0

    # A0 = np.diag([-1, -1])
    # B0 = np.diag([1.78018046, 0.95982973])
    c0 = np.array([0.17299905, 0.])
    A0 = np.zeros((2, 8))
    B0 = np.zeros((2, 8))

    act_pipe_cont, act_pipe_comm = Pipe()
    targ_pipe_cont, targ_pipe_comm = Pipe()
    state_queue = Queue()
    brew_event = Event()
    write_event = Event()
    comm_proc = CommProcess(act_pipe_comm, targ_pipe_comm, state_queue, brew_event)
    filter = LowPassFilter(0.7)
    cont_proc = MPC(
        act_pipe_cont,
        targ_pipe_cont,
        state_queue,
        write_event,
        brew_event,
        A,
        B,
        c,
        target_T,
        C,
        mhe_horizon,
        filter,
        H=25
    )
    # cont_proc = IndependentMIAC(
    #     filter, 7*np.eye(4*8+2), A0, B0, c0, act_pipe_cont, targ_pipe_cont, state_queue, write_event, brew_event,
    #     num_states=4
    # )
    # cont_proc = PID(filter, act_pipe_cont, targ_pipe_cont, state_queue, write_event, brew_event)
    # cont_proc = OnOff(filter, act_pipe_cont, targ_pipe_cont, state_queue, write_event, brew_event)
    comm_proc.start()
    cont_proc.start()
    input("Hit ENTER to start brewing.")
    brew_event.set()
    input("Hit ENTER to stop brewing.")
    brew_event.clear()
    input("Hit ENTER to stop data collection.")
    write_event.set()
