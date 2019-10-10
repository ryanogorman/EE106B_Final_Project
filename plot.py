import rospy

import matplotlib.pyplot as plt

def plot(num=300):
    # times = np.linspace(0, self.total_time, num=num)


    plt.figure()
    # plt.subplot(3,2,1)
    # plt.plot(times, target_positions[:,0], label='Desired')
    # plt.xlabel("Time (t)")
    # plt.ylabel("X Position")

    # plt.subplot(3,2,2)
    # plt.plot(times, target_velocities[:,0], label='Desired')
    # plt.xlabel("Time (t)")
    # plt.ylabel("X Velocity")
        
    # plt.subplot(3,2,3)
    # plt.plot(times, target_positions[:,1], label='Desired')
    # plt.xlabel("time (t)")
    # plt.ylabel("Y Position")

    # plt.subplot(3,2,4)
    # plt.plot(times, target_velocities[:,1], label='Desired')
    # plt.xlabel("Time (t)")
    # plt.ylabel("Y Velocity")
        
    # plt.subplot(3,2,5)
    # plt.plot(times, target_positions[:,2], label='Desired')
    # plt.xlabel("time (t)")
    # plt.ylabel("Z Position")

    # plt.subplot(3,2,6)
    # plt.plot(times, target_velocities[:,2], label='Desired')
    # plt.xlabel("Time (t)")
    # plt.ylabel("Z Velocity")
    plt.subplot(3,2,1)
    plt.plot(time, pos_x, 'g-')
    plt.xlabel("Time (t)")
    plt.ylabel("X Position")

    # plt.subplot(3,3,2)
    # plt.plot(time, vel_x, 'r-')
    # plt.xlabel("Time (t)")
    # plt.ylabel("X Velocity")

    plt.subplot(3,2,2)
    plt.plot(time, f_x, 'b-')
    plt.xlabel("Time (t)")
    plt.ylabel("X Force")

    plt.subplot(3,2,3)
    plt.plot(time, pos_y, 'g-')
    plt.xlabel("Time (t)")
    plt.ylabel("Y Position")

    # plt.subplot(3,3,5)
    # plt.plot(time, vel_y, 'r-')
    # plt.xlabel("Time (t)")
    # plt.ylabel("Y Velocity")

    plt.subplot(3,2,4)
    plt.plot(time, f_y, 'b-')
    plt.xlabel("Time (t)")
    plt.ylabel("Y Force")

    plt.subplot(3,2,5)
    plt.plot(time, pos_z, 'g-')
    plt.xlabel("Time (t)")
    plt.ylabel("Z Position")

    # plt.subplot(3,3,8)
    # plt.plot(time, vel_z, 'r-')
    # plt.xlabel("Time (t)")
    # plt.ylabel("Z Velocity")

    plt.subplot(3,2,6)
    plt.plot(time, f_z, 'b-')
    plt.xlabel("Time (t)")
    plt.ylabel("Z Force")

    plt.show()

if __name__ == '__main__':
	# with open('/home/cc/ee106b/sp19/class/ee106b-abm/ros_workspaces/final_project/src/r', 'r') as f:
	# 	lines = f.readlines()
	with open("record2", "r") as f:
		lines = f.readlines()
		print(lines)
	time = []
	pos_x = []
	pos_y = []
	pos_z = []
	vel_x = []
	vel_y = []
	vel_z = []
	f_x = []
	f_y = []
	f_z = []
	for line in lines[1:]:
		value = [float(x) for x in line.rstrip().split(',')]
		print(value)
		time.append(value[0])
		pos_x.append(value[1])
		pos_y.append(value[2])
		pos_z.append(value[3])
		vel_x.append(value[4])
		vel_y.append(value[5])
		vel_z.append(value[6])
		f_x.append(value[7])
		f_y.append(value[8])
		f_z.append(value[9])
	plot()