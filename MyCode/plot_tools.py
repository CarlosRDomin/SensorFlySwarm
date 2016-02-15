from datetime import datetime, timedelta
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

class MagnitudeControlLog:
    def __init__(self, str_magnitude="", pid_offset=0):
        self.lst_setpoint = []
        self.lst_measured = []
        self.lst_out_P = []
        self.lst_out_I = []
        self.lst_out_D = []
        self.pid_offset = pid_offset
        self.str_magnitude = str_magnitude

    def clear(self):
        self.lst_setpoint = []
        self.lst_measured = []
        self.lst_out_P = []
        self.lst_out_I = []
        self.lst_out_D = []

    def get_setpoint(self):
        return np.array(self.lst_setpoint)

    def get_measured(self):
        return np.array(self.lst_measured)

    def get_out_P(self):
        return np.array(self.lst_out_P)

    def get_out_I(self):
        return np.array(self.lst_out_I)

    def get_out_D(self):
        return np.array(self.lst_out_D)

    def update(self, measured, pid):
        self.pid_offset = pid.out_offs
        if self.str_magnitude.lower() == "thrust":
            self.update_long(-pid.SetPoint, -measured, pid.PTerm, pid.Ki * pid.ITerm, pid.Kd * pid.DTerm)
        elif pid.invert_error:
            self.update_long(pid.SetPoint, measured, -pid.PTerm, -pid.Ki * pid.ITerm, -pid.Kd * pid.DTerm)
        else:
            self.update_long(pid.SetPoint, measured, pid.PTerm, pid.Ki * pid.ITerm, pid.Kd * pid.DTerm)

    def update_long(self, setpoint, measured, out_P, out_I, out_D):
        self.lst_setpoint.append(setpoint)
        self.lst_measured.append(measured)
        self.lst_out_P.append(out_P)
        self.lst_out_I.append(out_I)
        self.lst_out_D.append(out_D)

    def plot(self, timestamp=None):
        if len(self.lst_measured) < 1:
            return None
        if timestamp is None:
            timestamp = [timedelta(seconds=i) for i in range(0, len(self.lst_measured))]
        timestamp = np.array(timestamp)
        if isinstance(timestamp[0], timedelta):
            timestamp += datetime.now().replace(hour=0, minute=0, second=0, microsecond=0)

        fig, ax1 = plt.subplots()
        ax1.plot(timestamp, self.get_setpoint(), 'r--', label=self.str_magnitude + ' SetPoint', linewidth=1.5)
        ax1.plot(timestamp, self.get_measured(), 'b', label=self.str_magnitude + ' measured', linewidth=2)
        ax1.set_xlim((min(timestamp), max(timestamp)))

        ax2 = ax1.twinx()
        ax2.plot(timestamp, self.pid_offset + self.get_out_P()+self.get_out_I()+self.get_out_D(), 'g', label=self.str_magnitude + ' PID output', linewidth=2)
        ax2.plot(timestamp, self.pid_offset + self.get_out_P(), 'm', label=self.str_magnitude + ' P component', linewidth=1)
        ax2.plot(timestamp, self.pid_offset + self.get_out_I(), 'c', label=self.str_magnitude + ' I component', linewidth=1)
        ax2.plot(timestamp, self.pid_offset + self.get_out_D(), 'k', label=self.str_magnitude + ' D component', linewidth=1)
        ax2.set_xlim(ax1.get_xlim())

        if self.str_magnitude.lower() != "thrust":
            ax1_ylim = ax1.get_ylim()
            ax2_ylim = ax2.get_ylim()
            ax1.set_ylim((min(ax1_ylim[0], ax2_ylim[0]), max(ax1_ylim[1], ax2_ylim[1])))
            ax2.set_ylim(ax1.get_ylim())

        ax1.grid(True)
        ax1.set_title(self.str_magnitude, fontsize='x-large', fontweight='bold')
        ax1.set_xlabel('Time (s)', fontsize='large', fontweight='bold')
        ax1.set_ylabel(self.str_magnitude + ' values', color='b', fontsize='large', fontweight='bold')
        for tl in ax1.get_yticklabels(): tl.set_color('b')
        ax2.set_ylabel(self.str_magnitude + ' PID output', color='g', fontsize='large', fontweight='bold')
        for tl in ax2.get_yticklabels(): tl.set_color('g')
        ax1.legend(fontsize='medium', loc='lower left')
        ax2.legend(fontsize='medium', loc='lower right')

        plt.ion()
        ax1.xaxis.set_major_formatter(mdates.DateFormatter('%M:%S'))
        fig.autofmt_xdate()  # Tilt x-ticks to be more readable
        fig.set_tight_layout(True)
        figManager = plt.get_current_fig_manager()
        figManager.window.showMaximized()
        fig.show()
        fig.savefig('log/log_' + self.str_magnitude + '_' + datetime.now().strftime("%m-%d_%H-%M-%S") + '.pdf')

        return fig


class OverallControlLog:
    def __init__(self):
        self.roll = MagnitudeControlLog("Roll")
        self.pitch = MagnitudeControlLog("Pitch")
        self.yaw = MagnitudeControlLog("Yaw")
        self.thrust = MagnitudeControlLog("Thrust")
        self.lst_timestamp = []

    def clear(self):
        self.roll.clear()
        self.pitch.clear()
        self.yaw.clear()
        self.thrust.clear()
        self.lst_timestamp = []

    def update(self, roll_measured, roll_PID, pitch_measured, pitch_PID, yaw_measured, yaw_PID, thrust_measured, thrust_PID, timestamp=None):
        if timestamp is None:
            timestamp = datetime.now()
        self.lst_timestamp.append(timestamp)
        self.roll.update(roll_measured, roll_PID)
        self.pitch.update(pitch_measured, pitch_PID)
        self.yaw.update(yaw_measured, yaw_PID)
        self.thrust.update(thrust_measured, thrust_PID)

    def update_long(self, roll_setpoint, roll_measured, roll_out_P, roll_out_I, roll_out_D,
                    pitch_setpoint, pitch_measured, pitch_out_P, pitch_out_I, pitch_out_D,
                    yaw_setpoint, yaw_measured, yaw_out_P, yaw_out_I, yaw_out_D,
                    thrust_setpoint, thrust_measured, thrust_out_P, thrust_out_I, thrust_out_D, timestamp=None):
        if timestamp is None:
            timestamp = datetime.now()
        self.lst_timestamp.append(timestamp)
        self.roll.update_long(roll_setpoint, roll_measured, roll_out_P, roll_out_I, roll_out_D)
        self.pitch.update_long(pitch_setpoint, pitch_measured, pitch_out_P, pitch_out_I, pitch_out_D)
        self.yaw.update_long(yaw_setpoint, yaw_measured, yaw_out_P, yaw_out_I, yaw_out_D)
        self.thrust.update_long(thrust_setpoint, thrust_measured, thrust_out_P, thrust_out_I, thrust_out_D)

    def plot(self, wait=True):
        self.roll.plot(self.lst_timestamp)
        self.pitch.plot(self.lst_timestamp)
        self.yaw.plot(self.lst_timestamp)
        self.thrust.plot(self.lst_timestamp)
        if wait:
            plt.waitforbuttonpress()


if __name__ == '__main__':
    from random import random
    log = MagnitudeControlLog("Roll")
    for x in range(0, 100):
        log.update_long(5, 5-random(), 10*random()-2, 3*random()-1.5, random())
    log.plot()
    plt.waitforbuttonpress()
