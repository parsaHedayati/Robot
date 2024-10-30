import numpy as np

from ikpy.chain import Chain
from ikpy.utils import plot
import matplotlib.pyplot as plt

spider1_chain = Chain.from_json_file("urdf/spider_f1.json")
spider2_chain = Chain.from_json_file("urdf/spider_f2.json")
spider3_chain = Chain.from_json_file("urdf/spider_b1.json")
spider4_chain = Chain.from_json_file("urdf/spider_b2.json")




target_position1 = [ 1, -0.6, 1]
target_position2 = [ 0.2, -0.6, 0]
target_position3 = [ 0.4, -0.6, 0]
target_position4 = [ 0.5, -0.6, 0]



print("The angles of each joints for front-1 are : ", spider1_chain.inverse_kinematics(target_position1))

print("The angles of each joints for front-2 are : ", spider2_chain.inverse_kinematics(target_position1))

print("The angles of each joints for bootom-1 are : ", spider3_chain.inverse_kinematics(target_position1))

print("The angles of each joints for bottom-2 are : ", spider4_chain.inverse_kinematics(target_position4)) 



# 3D Plot setup
from mpl_toolkits.mplot3d import Axes3D
fig, ax = plot.init_3d_figure()

# Plot the robot in its default position (all joints at 0)
spider1_chain.plot([0] * len(spider1_chain), ax)
spider2_chain.plot([0] * len(spider2_chain), ax)
spider3_chain.plot([0] * len(spider3_chain), ax)
spider4_chain.plot([0] * len(spider4_chain), ax)  
 




# If you are using a `baxter_head_chain` (but it’s unclear what this is referring to)
# Commenting this out for now because there's no definition for `baxter_head_chain` in your code
# baxter_head_chain.plot([0] * (4 + 2), ax)

# Add a legend
ax.legend()

# Show the plot
plt.show()