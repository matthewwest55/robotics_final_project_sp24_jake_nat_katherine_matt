import numpy as np

# Skip first row (header) with skiprows
joint_positions_csv = np.loadtxt("test_2_2.5_1.5-3.5.csv", delimiter=",", skiprows=1)

# knowing the matrix is a square, we can take the sqaure root to get its dimensions
square_dim = int(np.sqrt(len(joint_positions_csv)))

# The file is ordered from the bottom-right corner to the top-right
# Then, it moves left one position and goes again
# To make the matrix start at the top-left, we will reverse the ordering
matrix = np.ndarray((21, 21), dtype=np.ndarray)
for col in range(square_dim - 1, -1, -1):
    for row in range(square_dim - 1, -1, -1):
        # print("Col: " + str(col * square_dim) + " row: " + str(row) + " final: " + str(col*square_dim + row))
        # print(type(joint_positions_csv[(col*square_dim) + row][3:7]))
        matrix[square_dim - 1 - col][square_dim - 1 - row] = joint_positions_csv[(col*square_dim) + row][3:7]


print(matrix[20][20])
# print(joint_positions_csv[3][3:7])