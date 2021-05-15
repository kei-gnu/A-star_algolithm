import numpy as np


# ノードをclassで保存
class Node:
    # クラスの初期化。最初のノードは親もない、場所もない。
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        
        self.g = 0
        self.h = 0
        self.f = 0

# 経路を作成する関数
def return_path(node):
    # 出力経路リストの初期化
    path = []
    current = node
    # 現在のノードはスタートまで（親はNone）に迷路の場所(position)を保存
    while current is not None:
        # print(f"親ノード;{current.position}")
        path.append(current.position)
        current = current.parent
    
    # 作成した経路はゴールノードからスタートノードまでなので逆方向にする
    path = path[::-1]
    
    # 戻り値は作成した経路
    return path

# 経路を出力する関数 
def print_return_path(path, goal):
    if path:
        print("Path to goal:")
        path_string = ""
        for position in path:
            path_string += str(position)
            if position != goal:
                path_string += " -> "
        print(path_string)
        print("Next AI move:", path[0], "->", path[1])
    else:
        # 解がない場合、失敗を報告
        print("Search unsuccessful. No path to goal.")
    
# コストの計算を開始する関数
def move_cost_num(current_node, child_node, cost):
    current = current_node.position
    child = child_node.position
    # 経路コストを保存する変数
    next_node_cost = 0
    for c in cost:
        now_node = c[:2]    # 現在のノードの場所
        next_node = c[2:4]   # 次に移動するノードの場所
        # 位置の場所が一致したらnext_node_costに保存
        if ((current == now_node and child == next_node) or 
            (current == next_node and child == now_node)):
            next_node_cost = c[4] + current_node.g
            return next_node_cost

# ヒューリスティックを計算
def heuristics_num(current, goal):
    current_node = current.position
    goal_node = goal.position
    heuristics = abs((goal_node[0] - current_node[0])) + abs((goal_node[1] - current_node[1]))
    return heuristics

# f値を計算
def f_num(heuristics, cost):
    return heuristics + cost

# ダイスクトラアルゴリズム実行する関数
def a_star_algolithm(maze, start, goal, move_cost):
    # スタートノードとゴールノードを作成
    start_node = Node(None, start)
    goal_node = Node(None, goal)
    
    # ヒューリスティック値を追加
    start_node.h = heuristics_num(start_node, goal_node)
    # f値を取得
    start_node.f = f_num(start_node.h, start_node.g)
    # まだ訪問されていないノードと既に訪問されたノードを初期化
    queue = []
    # 既に訪問されたノードは同じノードをもう1回訪問しないためのリスト
    visited_nodes = []
    
    # スタートノードはまだ訪問されていないので未訪問リストに保存
    queue.append(start_node)
    
    # 2次元の迷路の可能な移動
    move = [[-1, 0],    # 上
            [0, -1],    # 左
            [1, 0],     # 下
            [0, 1]]     # 右
    
    # 迷路の行と列の数を把握
    row_no, column_no = np.shape(maze)
    
    # ゴールノードを発見するまでのループ
    while len(queue) > 0:
        # 一番コストが低いノードを取得する
        next_f_cost = []
        for cost_f in queue:
            next_f_cost.append(cost_f.f)
        next_f_cost.sort()
        # print(f"next_f_cost:{next_f_cost}")
        # print()
        
        index = 0
        for q in queue:
            if q.f == next_f_cost[0]:
                break
            index += 1
        
        # 待ち行列からコストが引くノードを取得し、削除
        current_node = queue.pop(index)
        #  次に展開するノードを既に訪問されたノードに追加
        visited_nodes.append(current_node)
        # print(current_node.position)
        
        
        # 展開のために選んだノードはゴールノードならば探索終了。その場合、戻り値はスタートからゴールまでの経路（return_path関数で作成)
        if current_node.position == goal_node.position:
            print("A* search success!!")
            print("Path cost:", current_node.g)
            return return_path(current_node)
        
        # 可能な移動先のノードを保存するためのリストを初期化
        children = []
        
        # ノードを展開する。すべての可能な行動を把握。この迷路ゲームの場合、4つの方向は可能（上、左、下、右）
        for new_position in move:
            # 次の場所を把握する
            node_position = [current_node.position[0] + new_position[0],
                            current_node.position[1] + new_position[1]]
            
            # 迷路から出ていないことを確認
            if (node_position[0] > (row_no -1) or
                node_position[0] < 0 or
                node_position[1] > (column_no -1) or 
                node_position[1] < 0):
                continue
            
            # 壁の確認
            if maze[node_position[0]][node_position[1]] != 0:
                continue
            
            # 可能な移動先。その移動先のノードを作成
            # 子ノードの経路コストを仮で保存
            new_node = Node(current_node, node_position)
            # new_node.g = 0
            # 可能な移動先のノードを保存（リストに追加）
            children.append(new_node)
            
            # 全ての作成された移動先のノードに対して、既に訪問されたかどうか、未訪問ノードの中にすでにあるかどうかを確認
            for child_node in children:
                # 子ノードの場所は既に訪問されたら追加しない
                for visited in visited_nodes:
                    if child_node.position == visited.position:
                        break
                
                else:
                    # 子ノードの経路コストを計算
                    child_node.g = move_cost_num(current_node, child_node, move_cost)
                    # 子ノードのヒューリスティック値を計算
                    child_node.h = heuristics_num(child_node, goal_node)
                    # 子ノードのf値を計算
                    child_node.f = f_num(child_node.h, child_node.g)
                
                # queueの中身を確認
                    for node in queue:
                        # 追加候補がqueueに入っているかつ、移動距離がqueueより小さい時
                        if (child_node.position == node.position):
                            # queueを更新
                            if node.g > child_node.g:
                                print(999)
                                node.g = child_node.gq
                                node.parent = current_node
                            break
                    else:
                        queue.append(child_node)
                    
# 迷路の情報を入力
def read_maze_file() :
    maze_file = "./maze2.txt"
    fileobj = open(maze_file)
    maze_info = fileobj.read()
    return maze_info

# 迷路の情報を取得する
def create_maze_info(maze_string):
    maze_row = []
    maze_info = []
    # char : 一文字の文字列を表す
    for maze_char in maze_string:
        if maze_char == '\n':
            # 一つの行を変更が完了。保存してから次の行の整数変更
            maze_info.append(maze_row)
            maze_row = []
        elif maze_char == 'W':
            # 文字列Wを1に変更する
            maze_row.append(1)
        elif maze_char == '0':
            maze_row.append(0)
    return maze_info

# スタートとゴールの情報を取得する関数
def get_start_goal_info(start_goal_string):
    start, goal = [], []
    # 余計なスペースと改行を削除
    start_goal_info = start_goal_string.split()
    # 縦と横の情報を整数に変更し、リストに保存
    start.append(int(start_goal_info[0]))
    start.append(int(start_goal_info[1]))
    goal.append(int(start_goal_info[2]))
    goal.append(int(start_goal_info[3]))
    return start, goal

# コストを取得する関数
def move_cost_int_info(cost_string):
    cost_char_info = cost_string.split()
    n = 5
    cost_int_info = [int(c) for c in cost_char_info]
    cost_int_split = [cost_int_info[ld:ld + n] for ld in range(0, len(cost_char_info), n)]
    return cost_int_split

def print_maze(maze, start, goal):
    print("Maze:")
    output_row = ""
    row_no = 0
    column_no = 0
    for row in maze:
        for entry in row:
            if start[0] == row_no and start[1] == column_no:
                output_row += "{:>3}".format("S")
            elif goal[0] == row_no and goal[1] == column_no:
                output_row += "{:>3}".format("G")
            else:
                output_row += ("{:>3}".format(entry))
            column_no += 1
        print(output_row)
        output_row = ""
        row_no += 1
        column_no =0


# 関数を実行する関数
def main():
    maze_string = read_maze_file()
    # ファイルの文字列を迷路情報とスタートとゴール情報を分ける
    maze_string_list = maze_string.split("#")
    # 経路コストの情報を$で分ける
    move_cost_list_string = maze_string.split("$")
    # 迷路の情報だけを取得
    maze_info_string = maze_string_list[0]
    # スタートとゴールの情報を取得
    start_goal_info_string = maze_string_list[1]
    # コストの情報を取得
    move_cost_info_string = move_cost_list_string[1]

    # 迷路の情報を文字から整数に変更する
    maze = create_maze_info(maze_info_string)
    
    # スタートの場所とゴールの場所を把握
    start, goal = get_start_goal_info(start_goal_info_string)
    # コストの情報を整数に変更
    maze_cost_info_int = move_cost_int_info(move_cost_info_string)
    # print(maze_cost_info_int)
    # 迷路を出力
    print_maze(maze, start, goal)
    
    # Dijkstraを実行
    print("Starting A* search...")
    path = a_star_algolithm(maze, start, goal, maze_cost_info_int)
    
    # 経路を出力
    print_return_path(path, goal)

if __name__ == '__main__':
    main()