import datetime as dt
from abc import ABCMeta, abstractmethod
from multiprocessing import Event, Pipe, Process, Queue

import cvxpy as cp
import numpy as np
import pause

from lib.serial_courier import SerialCourier

FREQ   = 20 #Hz
PERIOD = 1/FREQ
DELTA = dt.timedelta(seconds=PERIOD) #seconds
PERIODS_PER_FRAME = 10

PRE_INF_DUR = 10
RAMP_DUR = 5
PRE_INF_LEVEL = 1/5
RAMP_LEVEL = 3/5
FLOW_TARG = 2

class CommProcess(Process):
    def __init__(self, act_pipe, targ_pipe, state_queue, brew_event, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.act_pipe = act_pipe
        self.targ_pipe = targ_pipe
        self.state_queue = state_queue
        self.brew_event = brew_event
        self.comms = SerialCourier()
        self.action = [0.,0.]
        self.targets = [95., 9., 0.]

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
                self.action[1] = 0.
                self.comms.close_valve()
            else:
                brew_time = (next-start).total_seconds()

            self.comms.take_action(self.action[0],self.action[1])
            if i % PERIODS_PER_FRAME == 0: self.comms.refresh_display(
                'ESPRESSO',
                self.targets[0],
                self.targets[1],
                self.targets[2],
                brew_time
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
    def __init__(self, act_pipe, targ_pipe, state_queue, brew_event, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.act_pipe = act_pipe
        self.targ_pipe = targ_pipe
        self.state_queue = state_queue
        self.brew_event = brew_event
        self.state = np.zeros(4)

        self.targets = [94.5, 9., 0.]
        self.flow_coefs = [6.4634e+02, -7.0024e+01,  4.6624e+00, -1.9119e-01]

    @abstractmethod
    def calc_flow(self):
        pass

    @abstractmethod
    def temp_control(self, secs):
        pass

    @abstractmethod
    def flow_control(self, secs):
        pass

    @abstractmethod
    def run(self):
        pass

class OnOff(Controller):
    def calc_flow(self):
        pass

    def temp_control(self, secs):
        if self.state[0] < self.targets[0]:
            return 1.
        else:
            return 0.

    def flow_control(self, secs):
        if self.state[1] < self.targets[1]:
            return 1.
        else:
            return 0.

    def run(self):
        start = dt.datetime.now()
        next = start
        while True:
            while not self.state_queue.empty():
                self.state = self.state_queue.get()

            action = [0., 0.]
            action[0] = self.temp_control((next-start).total_seconds())

            if self.brew_event.is_set():
                action[1] = self.flow_control((next-start).total_seconds())
            else:
                self.targets[2] = 0
                start = next

            self.act_pipe.send(action)
            self.targ_pipe.send(self.targets)
            next += DELTA
            pause.until(next)

class PID(Controller):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.heat_last_error = None
        self.heat_last_integral = 0.
        self.pump_last_error = None
        self.pump_last_integral = 0.

    def calc_flow(self):
        full_flow = sum([self.flow_coefs[i] * self.state[1]**i for i in range(len(self.flow_coefs))]) / 60
        return full_flow * self.state[3]

    def temp_control(self, secs):
        target = self.targets[0]
        curr_temp = self.state[0]

        alpha = 1/15
        beta = 1/5000
        gamma = 1/20

        error = target - curr_temp

        proportional = error
        derivative = 0.
        integral = self.heat_last_integral

        if not (self.heat_last_error is None):
            derivative = (error - self.heat_last_error) / PERIOD
            integral += self.heat_last_error * PERIOD

        self.heat_last_error = error
        return alpha*proportional + beta*integral + gamma*derivative

    def flow_control(self, secs):
        target = self.targets[1]
        curr_temp = self.state[1]

        alpha = 1/15
        beta = 1/5000
        gamma = 1/20

        error = target - curr_temp

        proportional = error
        derivative = 0.
        integral = self.pump_last_integral

        if not (self.pump_last_error is None):
            derivative = (error - self.pump_last_error) / PERIOD
            integral += self.pump_last_error * PERIOD

        self.pump_last_error = error
        return alpha*proportional + beta*integral + gamma*derivative

    def run(self):
        start = dt.datetime.now()
        next = start
        while True:
            while not self.state_queue.empty():
                self.state = self.state_queue.get()

            action = [0., 0.]
            action[0] = self.temp_control((next-start).total_seconds())

            if self.brew_event.is_set():
                action[1] = self.flow_control((next-start).total_seconds())
            else:
                self.targets[2] = 0
                start = next

            self.act_pipe.send(action)
            self.targ_pipe.send(self.targets)
            next += DELTA
            pause.until(next)


class IndependentMIAC(Controller):
    def __init__(self, filter, P0, A0, B0, c0, num_states=1, *args, **kwargs):
        super().__init__(*args, **kwargs)
        assert P0.shape[0] == num_states * 8 + 2
        self.P = P0
        self.ahat = np.hstack((A0, B0, c0.reshape((-1, 1)))).flatten()
        self.Ahat = A0
        self.Bhat = B0
        self.chat = c0

        self.filter = filter
        self.targets = np.array(self.targets)
        self.state = None
        self.last_state = np.zeros((4, 0))

    def calc_flow(self):
        pass
        
    def temp_control(self, secs):
        # Implement MRAC temperature control method here
        pass

    def flow_control(self, secs):
        # Implement MRAC flow control method here
        pass

    def update_model(self):
        if self.last_state.shape[1] == num_states:
            phi = np.kron(np.concatenate((self.last_state.flatten(), [1])), np.eye(2))
            deriv = FREQ * (self.state[:2] - self.last_state[:2, -1])
            K = np.linalg.solve(np.eye(2) + phi @ self.P @ phi.T, phi @ self.P.T).T
            self.ahat += K @ (deriv - phi @ self.ahat)
            self.P = (np.eye(10) - K @ phi) @ self.P
        stacked = self.ahat.reshape((2, num_states * 4 + 1))
        self.Ahat = stacked[:, :num_states+1]
        self.Bhat = stacked[:, num_states+1:2*num_states+1]
        self.chat = stacked[:, 2*num_states+1]

    def compute_action(self):
        if self.state is None:
            return np.zeros(2)
        num_samples = 10 * FREQ
        r_cvx = cp.Variable((num_samples - 1, 2))
        traj = cp.Variable((num_samples, 2))

        traj_diff = traj - self.targets[None, :2]
        if not self.brew_event.is_set():
            traj_err = traj_diff @ np.diag([1, 0])
        else:
            traj_err =  traj_diff @ np.diag([1, 1])
        obj = cp.sum_squares(traj_err) + cp.sum_squares(r_cvx)

        endo = traj[:-1] @ self.Ahat.T + self.chat[None]
        exo = r_cvx @ self.Bhat.T
        constraints = [
            traj[1:] == traj[:-1] + PERIOD * (endo + exo),
            0 <= r_cvx,
            r_cvx <= 1,
            traj[0] == self.state[:2]
        ]
        if not self.brew_event.is_set():
            constraints += [
                r_cvx[:, 1] == 0
            ]

        prob = cp.Problem(cp.Minimize(obj), constraints)
        prob.solve()

        print(r_cvx.value[0])
        print(traj.value)
        print('\n' +'*'*20)
        return np.clip(r_cvx.value[0], 0, 1)

    def run(self):
        start = dt.datetime.now()
        next = start
        while True:
            while not self.state_queue.empty():
                obs = np.array(self.state_queue.get())
                obs[:2] = self.filter.apply(dt=PERIOD, value=obs[:2])
                print(obs)
                self.last_state, self.state = self.state, obs

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
        self.r = np.array([], dtype=np.dtype('float64'))
        self.rt = None

    def calc_flow(self):
        pass

    def temp_control(self, secs):
        # Implement MRAC temperature control method here
        return self.controls[0]

    def flow_control(self, secs):
        # Implement MRAC flow control method here
        return self.controls[1]

    def update_model(self):
        # Implement MRAC model update method here
        err = self.state - self.targets
        self.kx += -self.gamma * err * self.state * PERIOD * (1-(self.controls-0.5)**2)
        self.kr += -self.gamma * err * self.rt * PERIOD * (1-(self.controls-0.5)**2)
        self.controls = np.clip(self.kx * self.state + self.kr * self.rt, 0, 1)
        print(self.controls)

    def compute_reference(self):
        if len(self.r) == 0:
            num_samples = 60 * FREQ
            r_cvx = cp.Variable((num_samples - 1, 2))
            traj = cp.Variable((num_samples, 2))

            traj_diff = traj - self.targets[None]
            traj_err =  traj_diff @ np.diag([1, 1])
            obj = cp.sum_squares(traj_err) + cp.sum_squares(r_cvx)

            endo = traj[:-1] @ np.diag(self.Am) + self.cm[None]
            exo = r_cvx @ np.diag(self.Bm)
            constraints = [
                traj[1:] == traj[:-1] + PERIOD * (endo + exo),
                0 <= r_cvx,
                r_cvx <= 1,
                traj[0] == self.state
            ]

            prob = cp.Problem(cp.Minimize(obj), constraints)
            prob.solve()

            self.r = r_cvx.value
            self.rt, self.r = self.r[0], self.r[1:]
        self.rt, self.r = self.r[0], self.r[1:]
        if not self.brew_event.is_set():
            self.rt, self.r = self.r[0], self.r[1:]
            self.r[1] = 0.

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

            action = [0., 0.]
            action[0] = self.temp_control((next-start).total_seconds())

            if self.brew_event.is_set():
                action[1] = self.flow_control((next-start).total_seconds())
            else:
                start = next

            self.act_pipe.send(action)
            self.targ_pipe.send(list(self.targets) + [0])
            next += DELTA
            pause.until(next)

if __name__ == '__main__':
    A0 = np.diag([-1, -1])
    B0 = np.diag([1.78018046, 0.95982973])
    c0 = np.array([0.17299905, 0.])

    act_pipe_cont, act_pipe_comm = Pipe()
    targ_pipe_cont, targ_pipe_comm = Pipe()
    state_queue = Queue()
    brew_event = Event()
    comm_proc = CommProcess(act_pipe_comm, targ_pipe_comm, state_queue, brew_event)
    filter = LowPassFilter(0.8)
    cont_proc = IndependentMIAC(
        filter, 10*np.eye(10), A0, B0, c0, act_pipe_cont, targ_pipe_cont, state_queue, brew_event
    )
    comm_proc.start()
    cont_proc.start()
    input("Hit ENTER to start brewing.")
    brew_event.set()
    input("Hit ENTER to stop brewing.")
    brew_event.clear()
    # act_pipe_cont, act_pipe_comm = Pipe()
    # targ_pipe_cont, targ_pipe_comm = Pipe()
    # raw_state_queue = Queue()
    # filtered_state_queue = Queue()
    # brew_event = Event()

    # alpha_per_second = 0.8
    # filter_obj = LowPassFilter(alpha_per_second)

    # comm_proc = CommProcess(act_pipe_comm, targ_pipe_comm, raw_state_queue, brew_event)
    # filter_proc = FilterProcess(raw_state_queue, filtered_state_queue, filter_obj)
    # cont_proc = PID(act_pipe_cont, targ_pipe_cont, filtered_state_queue, brew_event)

    # comm_proc.start()
    # filter_proc.start()
    # cont_proc.start()
