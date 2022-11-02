import random as r

# define file names
print("enter start config file name : ")
start_conf_file = sys.argv[1] # 'test/start.txt'  # 'Sample_Start_Configuration.txt'
print("enter goal config file name : ")
goal_conf_file = sys.argv[2] # 'test/goal.txt'  # 'Sample_Goal_Configuration.txt'


def get_input(filename):
    conf = []
    fo = open(filename, "r")
    lines = fo.readlines()
    for l in lines:
        conf.append(l.strip().split())
    fo.close()
    return conf


start_configuration = get_input(start_conf_file)
goal_configuration = get_input(goal_conf_file)


def writedata(res, mode='w'):
    f = open("Output.txt", mode)
    f.writelines(res)
    f.close()


def numberOfTileMisplacement(start_conf):
    mat2 = goal_configuration
    n = len(mat2)
    nr_of_tiles = 0
    for i in range(0, n):
        for j in range(0, n):
            if i >= len(start_conf) or j >= len(start_conf[0]): continue
            if start_conf[i][j] != mat2[i][j] and start_conf[i][j] != '-':
                nr_of_tiles += 1
    return nr_of_tiles


def h(conf):
    return numberOfTileMisplacement(conf)
    # return totalManhattanDistance(conf)


def h_m(conf):
    return totalManhattanDistance(conf)


def totalManhattanDistance(start_conf):
    goal_conf = goal_configuration
    n = len(start_conf)
    start_conf = [i for row in start_conf for i in row]
    goal_conf = [i for row in goal_conf for i in row]

    manh_dist = 0
    for t in range(len(start_conf)):
        if (start_conf[t] != '-'):
            g_index = goal_conf.index(start_conf[t])
            manh_dist += abs(g_index // n - t // n) + abs(g_index % n - t % n)
    return manh_dist


class State:
    def __init__(self, conf, parent, g):
        self.conf = conf
        self.parent = parent
        self.g = g
        self.h = h(conf)

    def f(self):
        return self.g + self.h

    def f_m(self):
        return self.g + h_m(self.conf)


open_list = []
closed_list = []
output = None


def calc_child(cur):
    blank1 = None
    blank2 = None
    size = len(cur.conf)
    states = []

    for i in range(size):
        for j in range(size):
            if cur.conf[i][j] == '-':
                if blank1 == None:
                    blank1 = (i, j)
                else:
                    blank2 = (i, j)
    x1, y1 = blank1
    x2, y2 = blank2
    # [swap with, x val, y val]
    row_swaps = [[x1 - 1, x1, y1], [x1 + 1, x1, y1], [x2 - 1, x2, y2], [x2 + 1, x2, y2]]
    col_swaps = [[y1 - 1, x1, y1], [y1 + 1, x1, y1], [y2 - 1, x2, y2], [y2 + 1, x2, y2]]
    row_swaps.reverse()
    col_swaps.reverse()

    for r in row_swaps:
        temp = [x[:] for x in cur.conf]
        if 0 <= r[0] <= size - 1:
            temp[r[0]][r[2]], temp[r[1]][r[2]] = temp[r[1]][r[2]], temp[r[0]][r[2]]
            st = State(temp, cur, cur.g + 1)
            states.append(st)

    for c in col_swaps:
        temp = [x[:] for x in cur.conf]
        if 0 <= c[0] <= size - 1:
            temp[c[1]][c[2]], temp[c[1]][c[0]] = temp[c[1]][c[0]], temp[c[1]][c[2]]
            st = State(temp, cur, cur.g + 1)
            states.append(st)

    return states


def get_result(current_state):
    total_path = []
    while current_state.parent is not None:
        from_state = current_state.parent
        to_state = current_state
        size = len(current_state.conf)
        move = "("
        for row in range(size):
            for col in range(size):
                if from_state.conf[row][col] != to_state.conf[row][col] and from_state.conf[row][col] != '-':
                    move += from_state.conf[row][col] + ","
                    if row - 1 >= 0 and from_state.conf[row][col] == to_state.conf[row - 1][col]:
                        move += "up)"
                    if row + 1 <= (size - 1) and from_state.conf[row][col] == to_state.conf[row + 1][col]:
                        move += "down)"
                    if col - 1 >= 0 and from_state.conf[row][col] == to_state.conf[row][col - 1]:
                        move += "left)"
                    if col + 1 <= (size - 1) and from_state.conf[row][col] == to_state.conf[row][col + 1]:
                        move += "right)"
                    break
        total_path.append(move)
        current_state = current_state.parent
    total_path.reverse()
    return total_path


def get_optimum_state(flag):
    if flag == "hamming":
        min_state = open_list[0]
        min_f = open_list[0].f()

        for state in open_list[1:]:
            if state.f() < min_f:
                min_state = state
                min_f = state.f()
        return min_state
    else:
        min_state = open_list[0]
        min_f = open_list[0].f_m()

        for state in open_list[1:]:
            if state.f_m() < min_f:
                min_state = state
                min_f = state.f_m()
        return min_state


def AStarAlgorithm(flag="hamming"):
    open_list.append(State(start_configuration, None, 0))
    while len(open_list) > 0:
        current_state = get_optimum_state(flag)
        if current_state.conf == goal_configuration:
            expands = len(closed_list)
            closed_list.clear()
            open_list.clear()
            return (expands, current_state)
        open_list.remove(current_state)
        closed_list.append(current_state)

        for state in calc_child(current_state):
            # check if current state is in closed
            in_closed = False
            for i in closed_list:
                if i.conf == state.conf:
                    in_closed = True
                    break

            if in_closed:
                continue
            # check if current state is in _open
            in_open = False
            for i in open_list:
                if i.conf == state.conf:
                    in_open_state = i
                    in_open = True
                    break

            if not (in_open):
                open_list.append(state)
            else:
                if state.g < in_open_state.g:
                    in_open_state.g = state.g
                    in_open_state.parent = state.parent


expands, final1 = AStarAlgorithm("manhatton")  # manhattan
path = get_result(final1)
res = ', '.join(path)
writedata(res)

# ========================
#  T E S T I N G
# ======================


def generate_goal(children_, total, turn):
    chosen_child = children_[r.randint(0, len(children_) - 1)]
    children_ = calc_child(chosen_child)
    if total == turn:
        return chosen_child.conf
    return generate_goal(children_, total, turn + 1)


def get_start_goal():
    results = []
    for i in range(100):
        size_n = r.randint(5, 20)
        numbers = list(map(lambda a: str(a), list(range(1, size_n * size_n - 1))))
        numbers.append('-')
        numbers.append('-')
        r.shuffle(numbers)

        start_config = []
        for j in range(size_n):
            start_config.append(numbers[size_n * j:size_n * (j + 1)])

        start = State(start_config, None, 0)
        children = calc_child(start)

        goal_config = generate_goal(children, r.randint(20, 30), 1)

        if not goal_config: continue

        global start_configuration, goal_configuration
        start_configuration = start_config
        goal_configuration = goal_config
        expands1, final1 = AStarAlgorithm()  # hamming
        expands, final1 = AStarAlgorithm("manhatton")  # manhattan
        l = [expands1, expands, int(expands1) - int(expands)]
        results.append(l)

        # writedata(str(l) + "\n", "testOutputData.txt", "a")
        # writedata(str(start_config) + "," + str(goal_config) + "\n", "testData.txt", "a")

    return results



# load outputs to the data file
# results = get_start_goal()

# load outputs from data file
# results = []
# fo = open("testOutputData.txt", "r")
# lines = fo.readlines()
# for l in lines:
#     results.append(eval(l.strip()))
# fo.close()


#print output data
# j = 1
# for i in results:
#     ex1 = i[0]
#     ex = i[1]
#     dif = i[2]
#     print(j, "=>", ex1, ", ", ex, ": ", dif)
#     j += 1


# #calculations
# differences = []
# for i in results:
#     differences.append(i[2])
# meanDifference = sum(differences) / len(differences)
# print("\n\n\nMean of the differences : %f "%meanDifference)


# import scipy.stats as stats
# moves_manhattan = []
# moves_misplaced =[]
# for i in results:
#     moves_manhattan.append(i[1])
#     moves_misplaced.append(i[0])
# test_statistic,pvalue=stats.ttest_rel(moves_manhattan, moves_misplaced)
#
# print("Test Statistic value: %f" %test_statistic)
# print("P value: %f" %pvalue)
