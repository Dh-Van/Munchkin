import numpy as np

class Controller:
    def __init__(self, num_dof, arm_lengths):
        self.num_dof = num_dof
        self.arm_lengths = arm_lengths
        
    def forward_kinematics(self, theta_list):
        T = self.thetalist_to_transformation_matrix(theta_list)
        return T[0] @ T[1] @ T[2] @ T[3] @ np.array([0, 0, 0, 1])
    
    def velocity_kinematics(self, theta_list, velocity):
        thetadot = self.calc_damped_inverse_jacobian(theta_list) @ velocity
        return theta_list + thetadot
        
    def numerical_inverse_kinematics(self, starting_theta_list, ee_pos):
        ilimit = 1000
        tol = 0.01
        theta_list = starting_theta_list.copy()
                
        for _ in range(ilimit):
            curr_position = self.forward_kinematics(theta_list)
            error = ee_pos - curr_position[:3]
            
            if abs(max(error, key=abs)) < tol:
                break
            
            theta_list += self.calc_damped_inverse_jacobian(theta_list) @ error            
        
        return theta_list
        
    def generate_dh_table(self, theta_list):
        DH = np.zeros(shape = (4, 4))
        
        # theta, d, alpha, a
        DH[0] = [theta_list[0], self.arm_lengths[0], np.pi / 2, 0]
        DH[1] = [theta_list[1] + np.pi / 2, 0, 0, self.arm_lengths[1]]
        DH[2] = [theta_list[2], 0, 0, self.arm_lengths[2]]
        DH[3] = [theta_list[3], 0, 0, self.arm_lengths[3]]
        
        return DH
    
    def param_to_transformation_matrix(self, dh_params):
        theta, d, alpha, a = dh_params
        
        T = np.zeros(shape = (4, 4))
        T[0] = [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)]
        T[1] = [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)]
        T[2] = [0, np.sin(alpha), np.cos(alpha), d]
        T[3] = [0, 0, 0, 1]
        
        return T
    
    def thetalist_to_transformation_matrix(self, theta_list):
        DH = self.generate_dh_table(theta_list)
        T = np.zeros(shape = (self.num_dof, 4, 4))
        
        for i in range(self.num_dof):
            params_i = DH[i]
            T[i] = self.param_to_transformation_matrix(params_i)
        
        return T
    
    def cumulative_transformation_matrix(self, theta_list):
        T = self.thetalist_to_transformation_matrix(theta_list)
                
        T_cumulative = [np.eye(4)]
        for i in range(self.num_dof):
            T_cumulative.append(T_cumulative[-1] @ T[i])
            
        return T_cumulative

    
    def calc_jacobian(self, theta_list):
        T = self.thetalist_to_transformation_matrix(theta_list)
        J = np.zeros(shape = (3, self.num_dof))
        
        T_cumulative = [np.eye(4)]
        for i in range(self.num_dof):
            T_cumulative.append(T_cumulative[-1] @ T[i])
            
        translation = np.array([0, 0, 0, 1])
        
        for i in range(self.num_dof):
            T_i = T_cumulative[i]
            T_f = T_cumulative[-1]
            
            r = (T_f @ translation - T_i @ translation)[:3]
            z = T_i[:3, :3] @ np.array([0, 0, 1])
            
            J[:, i] = np.cross(z, r)
            
        return J
            
            
    def calc_damped_inverse_jacobian(self, theta_list, damping_factor = 0.1):
        J = self.calc_jacobian(theta_list)
        JT = np.transpose(J)
        I = np.eye(3)
        
        return JT @ np.linalg.inv(J @ JT + (damping_factor ** 2) * I)


    def calc_joint_positions(self, theta_list):
        points = np.zeros((5, 3))
        points[0] = np.array([0, 0, 0])
        
        T_cumulative = self.cumulative_transformation_matrix(theta_list)
            
        for i in range(1, 5):
            point = T_cumulative[i] @ np.array([0, 0, 0, 1])
            points[i] = point[:3]
            
        return points
    
    def camera_to_pos(self, theta_list, camera_pos, offset = [1.5, 2.0, 1.2]):
        
        camera_transformation_matrix = np.eye(4)
        camera_transformation_matrix[0, 3] = offset[0]
        camera_transformation_matrix[1, 3] = offset[1]
        camera_transformation_matrix[2, 3] = offset[2]
        
        T_final = self.cumulative_transformation_matrix(theta_list)[-1]
        
        p_camera = np.append(camera_pos, 1)
        
        return  (T_final @ camera_transformation_matrix @ p_camera)[:3]
    
    def create_trajectory(self, p1, p2):
        resolution = 100
        
        t_values = np.linspace(0, 1, resolution)
        points = (1 - t_values[:, np.newaxis]) * p1 + t_values[:, np.newaxis] * p2
        
        return points
    
    def trajectory_to_theta_list(self, intial_thetalist, trajectory_points):
        theta_list = []
        prev_thetalist = intial_thetalist
        for point in trajectory_points:
            point_thetalist = self.numerical_inverse_kinematics(prev_thetalist, point)
            theta_list.append(point_thetalist)
            prev_thetalist = point_thetalist
            
        return theta_list
    
    def clip_angles(self, angle, max_angle=np.radians(240.0)):
        angle = angle % (2*np.pi)
        
        zero_dist = 2 * np.pi - angle
        max_dist = angle - max_angle
            
        clipped_angle = np.where(
            angle <= max_angle,
            angle,
            np.where(
                zero_dist < max_dist,
                0.0,
                max_angle
            )
        )
        
        return clipped_angle

