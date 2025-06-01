#!/usr/bin/env python
# coding: utf-8

# ## Quaternions
# 

# In the following exercise you'll implement functions to convert between Euler angles and quaternion representations. It's useful to be able to easily navigate back and forth between these representations because of their relative strengths. Quaternions are better for calculations, while Euler angles are far more intuitive.
# 
# Some messages coming from your drone in simulation (or in the real world) will represent orientation data as a quaternion, while others use Euler angles. So it's a good idea to be able to seamlessly handle both. 
# 
# The [`udacidrone` API imlementation](https://github.com/udacity/udacidrone/blob/master/udacidrone/connection/message_types.py#L189-L284) that you're using for the projects in this program already has these conversions implemented under the hood so that's a great place to start if you aren't sure how to complete this exercise!

# In[6]:


import numpy as np

def euler_to_quaternion(angles):
    roll = angles[0]
    pitch = angles[1]
    yaw = angles[2]
    
    # TODO: complete the conversion
    # and return a numpy array of
    # 4 elements representing a quaternion [a, b, c, d]
    _phi = roll/2
    s_phi = np.sin(_phi)
    c_phi = np.cos(_phi)
    
    _theta = pitch/2
    s_theta = np.sin(_theta)
    c_theta = np.cos(_theta)
    
    _psi = yaw/2
    s_psi = np.sin(_psi)
    c_psi = np.cos(_psi)
    
    a = c_phi*c_theta*c_psi + s_phi*s_theta*s_psi
    b = s_phi*c_theta*c_psi - c_phi*s_theta*s_psi
    c = c_phi*s_theta*c_psi + s_phi*c_theta*s_psi
    d = c_phi*c_theta*s_psi - s_phi*s_theta*c_psi
    
    quaternion = np.array([a, b, c, d])
    
    return quaternion
    
def quaternion_to_euler(quaternion):
    a = quaternion[0]
    b = quaternion[1]
    c = quaternion[2]
    d = quaternion[3]
    
    # TODO: complete the conversion
    # and return a numpy array of
    # 3 element representing the euler angles [roll, pitch, yaw]
    phi = np.arctan2(2*(a*b + c*d), (1 - 2*(b*b + c*c)))
    theta = np.arcsin(2*(a*c - d*b))
    psi = np.arctan2(2*(a*d + c*b), (1 - 2*(d*d + c*c)))
    
    angles = np.array([phi, theta, psi])
    
    return angles


# Test the conversion.

# In[7]:


euler = np.array([np.deg2rad(90), np.deg2rad(30), np.deg2rad(0)])

# should be [ 0.683  0.683  0.183 -0.183]
q = euler_to_quaternion(euler) 
print(q)

# should be [ 1.570  0.523  0.]
e = quaternion_to_euler(q)
print(e)
assert np.allclose(euler, quaternion_to_euler(q))


# Here's our [solution](/notebooks/Quaternions-Solution.ipynb)!

# In[ ]:




