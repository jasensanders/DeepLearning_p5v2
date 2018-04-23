import numpy as np
from physics_sim import PhysicsSim

class Task():
    """Task (environment) that defines the goal and provides feedback to the agent."""
    def __init__(self, init_pose=None, init_velocities=None, 
        init_angle_velocities=None, runtime=5., target_pos=None):
        """Initialize a Task object.
        Params
        ======
            init_pose: initial position of the quadcopter in (x,y,z) dimensions and the Euler angles
            init_velocities: initial velocity of the quadcopter in (x,y,z) dimensions
            init_angle_velocities: initial radians/second for each of the three Euler angles
            runtime: time limit for each episode
            target_pos: target/goal (x,y,z) position for the agent
        """
        # Simulation
        self.sim = PhysicsSim(init_pose, init_velocities, init_angle_velocities, runtime) 
        self.action_repeat = 3

        self.state_size = self.action_repeat * 6
        self.action_low = 100
        self.action_high = 900
        self.action_size = 4

        # Goal
        self.target_pos = target_pos if target_pos is not None else np.array([0., 0., 10.]) 

    def get_reward(self):
        """Uses current pose of sim to return reward."""
        reward = 1.-.3*(abs(self.sim.pose[:3] - self.target_pos)).sum()
        return reward

    def step(self, rotor_speeds):
        """Uses action to obtain next state, reward, done."""
        reward = 0
        pose_all = []
        for _ in range(self.action_repeat):
            done = self.sim.next_timestep(rotor_speeds) # update the sim pose and velocities
            reward += self.get_reward() 
            pose_all.append(self.sim.pose)
        next_state = np.concatenate(pose_all)
        return next_state, reward, done

    def reset(self):
        """Reset the sim to start a new episode."""
        self.sim.reset()
        state = np.concatenate([self.sim.pose] * self.action_repeat) 
        return state

class TakeOffTask():
    """Task (environment) that defines the goal and provides feedback to the agent."""
    def __init__(self, init_pose=None, init_velocities=None, 
        init_angle_velocities=None, runtime=5., target_pos=None):
        """Initialize a Task object.
        Params
        ======
            init_pose: initial position of the quadcopter in (x,y,z) dimensions and the Euler angles
            init_velocities: initial velocity of the quadcopter in (x,y,z) dimensions
            init_angle_velocities: initial radians/second for each of the three Euler angles
            runtime: time limit for each episode
            target_pos: target/goal (x,y,z) position for the agent
        """
        # Simulation
        self.sim = PhysicsSim(init_pose, init_velocities, init_angle_velocities, runtime) 
        self.action_repeat = 3

        self.state_size = self.action_repeat * 6
        self.action_low = 400
        self.action_high = 900
        self.action_size = 4

        # Goal
        self.target_pos = target_pos if target_pos is not None else np.array([0., 0., 10.])
        # Level flight not spinning in any direction
        self.target_angles = np.array([0.,0.,0.]) 
        self.target_Ang_V = np.array([0., 0., 0.])
        # We should only be going up. Only Z velocity is a target
        self.target_v = init_velocities if init_velocities is not None else np.array([0., 0., 10.]) 
        self.init_pose = init_pose

    def get_reward(self):
        """Uses current pose of sim to return reward."""
        # Reward version 1.5
        """R = self.sim.v[2]*np.cos(offAngleDiff)+.3*(abs(self.sim.pose[2]-self.init_pose[2])) -.7*offPoseDiff -.3*offAngleDiff
        -.4*(abs(self.sim.angular_v - self.target_Ang_V)).sum()
        # Normailize rewards scale to value between -1 and 1
        reward = np.tanh(R)"""
        
        # Reward Version 2
        """R = self.sim.v[2]*np.cos(offAngleDiff)+.3*(self.sim.pose[2]-self.init_pose[2]) -.7*offPoseDiff -.7*offAngleDiff
        -.7*(abs(self.sim.angular_v - self.target_Ang_V)).sum()"""
        
        # Reward Version 3 (Reward good things, punish bad things)
        """offPoseDiff =(abs(self.sim.pose[:3] - self.target_pos)).sum()
        offAngleDiff = (abs(self.sim.pose[3:] - self.target_angles)).sum() 
        angVelocDiff = (abs(self.sim.angular_v - self.target_Ang_V)).sum()
        veloDiff = (abs(self.sim.v - self.target_v)).sum()
        poseDiffPos = self.sim.pose[2]-self.init_pose[2]
        def velo_rewardPos(diff):
            zdiff = self.sim.pose[2] - self.target_pos[2]
            if zDiff < 0.1 and zDiff > -0.1:
                return 20
            else:
                return diff*1
        def inv(x):
            return 1/(0.02+x**2)
        def isNeg(x):
            return x<0
        
        R = velo_rewardPos(self.sim.v[2]-self.target_v[2]) + poseDiffPos + inv(angVelocDiff)*10 + inv(offAngleDiff)*10 +
        inv(offPoseDiff)*10 + inv(veloDiff)*10  
        
        # Normailize rewards scale to value between -1 and 1
        #reward = np.tanh(R)"""
        
        # Reward Version 4 (Where I found out that taking the sine of the radian angles was important becasue it gives distance off)
        speedZ = slf.sim.v[2]
        offAngleCosTerm = speedZ*(np.cos(self.sim.pose[3:])).sum()
        offAngleSinTerm = abs(speedZ*(np.sin(self.sim.pose[3:])).sum())
        
        # Reward Version 5 penalize theta
        #thetaPenalty = speedZ*np.sin(self.sim.pose[4])
        
        # Reward Version 6 theta is penalized already, penalize off position
        offPositionTerm = speedZ*(abs(self.target_pos - self.sim.pose[:3]).sum())
        # Sum the Reward/Penalty
        R= offAngleCosTerm - offAngleSinTerm - offPositionTerm 
        # Penalize Crashing
        if self.sim.pose[2] < self.target_pose[2]:
            R = -300
        
        reward = R
        return reward
    
    

    def step(self, rotor_speeds):
        """Uses action to obtain next state, reward, done."""
        reward = 0
        pose_all = []
        for _ in range(self.action_repeat):
            done = self.sim.next_timestep(rotor_speeds) # update the sim pose and velocities
            reward += self.get_reward() 
            pose_all.append(self.sim.pose)
        next_state = np.concatenate(pose_all)
        return next_state, reward, done

    def reset(self):
        """Reset the sim to start a new episode."""
        self.sim.reset()
        state = np.concatenate([self.sim.pose] * self.action_repeat) 
        return state