import requests
from uuid import uuid4
import time
import os
import random
SERVER_URL  = os.environ.get("sf-judge-server") or "http://127.0.0.1:5555"
print(SERVER_URL)
USERNAME = "算法挑战赛-AI平台"
SUBMISSION_ID = str(uuid4())


def class_init(map_info):
    
    '''存储agv相关信息'''
    class Agv:
        def __init__(self, agv_id, payload, cap, loc=[-1,-1], occupy=-1):
            self.loc = loc #列表类型，如[1,1]，表示坐标
            self.id = agv_id  #agv的id
            self.payload = payload #是否载货，初始化为-1
            self.cap = cap  #容量，能载几个
            self.occupy = occupy #是否正在前去/已经装载货物的编号，初始值为-


    '''存储货物相关信息'''
    class Cargo:
        def __init__(self, cargo_id, target, loc=[-1,-1], load=-1):
            self.loc = loc #列表类型，如[1,1]，表示坐标
            self.id = cargo_id  #cargo的id
            self.target = target  #目标货架
            self.load = load #是否正要被前往装载，或者已经被AVG装载，初始化为-1

    class Shelf:
        def __init__(self, shelf_id, payload, loc=[-1,-1]):
            self.loc = loc #列表类型，如[1,1]，表示坐标
            self.id = shelf_id  #shelf的id
            self.payload = payload
            
            
    class sf:
        
        def __init__(self, map_json):
            self.map_json = map_json
            self.ACTIONS_SEQ1 = []
            
            self.map_data = [] #存放map数据，先随机指定类型
            self.map_width = 0 #地图宽度
            self.map_height = 0 #地图高度
            self.max_steps = 0 #最大步数
            self.timeiout = 0 #单次决策最大时间
            
            

            self.shelfs = {} #存放货架相关信息,id作为key
            self.agvs = {} #存放Agv相关信息, id作为key
            self.cargos = {} #存放cargo相关信息， target作为key
            self.obstacles = set() #存放walls,以及其他物体的的坐标,都可以视为障碍物
            
            self.path_obstacles = set() #用于bfs的障碍推断
            self.agvs_paths = {} #用于保存各个agvs的运货路径

        '''
        优化方向：agv运输的次序, 序号优先，还是距离最近的优先行动，或者是随机顺序行动
        寻路方式，除了bfs，优化的bfs，A*算法
        寻找距离最短的agv去运输货物
        死锁处理方式
        '''


        '''通过地图的json文件构造地图,存放AGV货物等的信息'''
        def map_construct(self):
            self.map_data = self.map_json
            self.map_width = self.map_data['value']['map_attr']['width']
            self.map_height = self.map_data['value']['map_attr']['height']
            self.max_steps = self.map_data['value']['map_attr']['max_steps']
            self.timeiout = self.map_data['value']['map_attr']['timeout']

            '''存放地图中agv信息'''
            for agv_info in self.map_data['value']['map_state']['agvs']:

                agv_id = agv_info['id']
                payload = (-1 if agv_info['payload'] is None else agv_info['payload']) #如果初始shelf没有货物，则为-1
                cap = agv_info['cap']
                self.agvs[agv_id] = Agv(agv_id, payload, cap)

            '''存放地图中cargo信息'''
            for cargo_info in self.map_data['value']['map_state']['cargos']:
                cargo_id = cargo_info['id']
                target = cargo_info['target']
                self.cargos[cargo_id] = Cargo(cargo_id, target)
            
            '''存放地图中shelf信息'''
            for shelf_info in self.map_data['value']['map_state']['shelves']:
                shelf_id = shelf_info['id']
                payload = (-1 if shelf_info['payload'] is None else shelf_info['payload']) #如果初始shelf没有货物，则为-1
                self.shelfs[shelf_id] = Shelf(shelf_id, payload)
            
            '''将各个物体的位置信息加入'''
            for loc_info in self.map_data['value']['map_state']['map']:
            
                if loc_info['type'] == 'agv':

                    agv_id = loc_info['id']
                    x = loc_info['y']
                    y = loc_info['x']
                    self.agvs[agv_id].loc = [x, y]
                    self.obstacles.add((x, y))
                    
                elif loc_info['type'] == 'cargo':
                    cargo_id = loc_info['id']
                    x = loc_info['y']
                    y = loc_info['x']
                    self.cargos[cargo_id].loc = [x, y]
                    self.obstacles.add((x, y))
                    
                elif loc_info['type'] == 'shelf':
                    shelf_id = loc_info['id']
                    x = loc_info['y']
                    y = loc_info['x']
                    self.shelfs[shelf_id].loc = [x, y]
                    self.obstacles.add((x, y))
                else:
                    x = loc_info['y']
                    y = loc_info['x']
                    self.obstacles.add((x, y))
                

        '''对每一张地图根据指令执行AVG各项操作'''
        def map_process(self):
            self.map_data = self.map_json #获取地图号为map_id地图信息
            '''存放地图信息'''
            self.map_construct()
            self.process()
            

        '''
            寻找某一位置到另一位置的可行路径
            start: 原位置，如[0,0]
            end: 目的位置，如[10,10]
            obstacles:障碍物list 如((1,1), (0,0), (2,2))
            width:地图宽度
            height:地图高度
        '''

        def bfs(self, start, end):
            if end[0]<0 or end[0]>=self.map_height or end[1]<0 or end[1]>=self.map_width: #end位置不在地图内,地图的最短宽和高都为1
                # print("边界")
                # print(str(start) +" "+ str(end))
                return []
            start = tuple(start)
            end = tuple(end)  #元组化，从而可以作为字典的键
            queue = [start]   
            visited = set(self.path_obstacles) #visit与obstacles内存不同
            distance = {start: 0}     
            parent = {start: None}   

            while queue:  
                current = queue.pop(0)  
                #print(current)
                #visited.remove(current)
                if current == end:  
                    break

                for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:  
                    next_x = current[0] + dx
                    next_y = current[1] + dy
                    next_point = (next_x, next_y)

                    if next_point not in visited and (next_x >= 0) and (next_x < self.map_height) and (next_y >= 0) and (next_y < self.map_width):
                        visited.add(next_point) 
                        queue.append(next_point) 
                        distance[next_point] = distance[current] + 1 
                        parent[next_point] = current  

            if end not in parent:  
                return []
            
            path = [end]
            while path[-1] != start:
                path.append(parent[path[-1]])
            #self.path_obstacles.union(set(path))
            #print(self.path_obstacles)
            return list(reversed(path))


        '''检测是否完成该地图,即所有货物均放在了货架上'''
        def isComplete(self):
            for cargo_id in self.cargos:
                shelf_id = self.cargos[cargo_id].target
                if self.shelfs[shelf_id].payload != cargo_id:
                    return False
            return True



        def paths_init(self):
            #初始化agvs的路径
            for agv_id in range(len(self.agvs)):
                self.agvs_paths[agv_id] = []
            
                
        '''以随即顺序对所有的agv进行调度'''
        def forward(self):
            action_idx = len(self.ACTIONS_SEQ1)
            self.ACTIONS_SEQ1.append([])
            for i in range(len(self.agvs)): #本轮所有agv的操作字典
                self.ACTIONS_SEQ1[action_idx].append({})
            
            #max_steps -= 1#地图最大步数
            is_loc = True #判断是否死锁，即没有一个AGV能够运动,都要stay
            agvs_ids_list = [i for i in range(len(self.agvs))]
            #random.shuffle(agvs_ids_list) #AGV运输顺序为随机
            for agv_id in agvs_ids_list:
                if self.agvs_paths[agv_id] == []:
                    #print('路径为空')
                    #agvs没有要行走的路径不动
                    self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "STAY"
                    self.agvs_paths[agv_id]=[]
                    continue
                    
                elif self.agvs_paths[agv_id][0] == self.agvs_paths[agv_id][-1]: #AGV到达了终点
                    #print('agv到达终点')
                    is_loc = False
                    if self.agvs[agv_id].payload != -1:
                        #卸货
                        
                        cargo_id = self.agvs[agv_id].payload
                        self.shelfs[self.cargos[cargo_id].target].payload = cargo_id
                        shelf_id = self.cargos[cargo_id].target
                        self.cargos[cargo_id].load = -1 #货物卸载不在AGV上了
                        self.agvs[agv_id].occupy = -1#agv卸货，没有货物了
                        self.agvs[agv_id].payload = -1#agv当前没有货物运输
                        self.cargos[cargo_id].loc = self.shelfs[self.cargos[cargo_id].target].loc #货物的位置发生变化，被卸货
                        #obtucles不需要改变，因为是set并且没有新的obtacles加入
                        self.agvs_paths[agv_id] = [] #把当前路径清空，已经到达终点
                        
                        self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "DELIVERY"
                        #判断卸货方向
                        direct = ""
                        shelf_loc = self.shelfs[shelf_id].loc
                        agv_loc = self.agvs[agv_id].loc
                        if shelf_loc[0]-agv_loc[0] != 0:
                            if shelf_loc[0]-agv_loc[0] == 1:
                                direct = "DOWN"
                            else:
                                direct = "UP"
                        elif shelf_loc[1] != agv_loc[1]:
                            if shelf_loc[1]-agv_loc[1] == 1:
                                direct = "RIGHT"
                            else:
                                direct = "LEFT"
                        self.ACTIONS_SEQ1[action_idx][agv_id]["dir"] = direct
                        print('agv'+str(agv_id)+'卸货货物'+str(cargo_id)+'到货架'+str(shelf_id))
                        continue
                    
                    if self.agvs[agv_id].occupy != -1:
                        #装货
                        cargo_id = self.agvs[agv_id].occupy
                        cargo_loc = self.cargos[cargo_id].loc
                        
                        is_remove = True
                        for shelf in self.shelfs:
                            if self.shelfs[shelf].loc == self.cargos[cargo_id].loc:
                                is_remove = False
                        if is_remove:
                            self.obstacles.remove(tuple(self.cargos[cargo_id].loc))
                        self.cargos[cargo_id].loc = self.agvs[agv_id].loc #货物装上agv
                        self.agvs[agv_id].payload = cargo_id #agvs开始运输,
                        self.agvs_paths[agv_id] = [] #agv已经到达终点，等待deliver分配新的路线
                        
                        self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "PICKUP"
                        #判断装货方向
                        direct = ""
                        agv_loc = self.agvs[agv_id].loc
                        if cargo_loc[0]-agv_loc[0] != 0:
                            if cargo_loc[0]-agv_loc[0] == 1:
                                direct = "DOWN"
                            else:
                                direct = "UP"
                        elif cargo_loc[1] != agv_loc[1]:
                            if cargo_loc[1]-agv_loc[1] == 1:
                                direct = "RIGHT"
                            else:
                                direct = "LEFT"
                        self.ACTIONS_SEQ1[action_idx][agv_id]["dir"] = direct
                        print('agv'+str(agv_id)+'装货'+str(cargo_id))
                        continue
                    
                    else:
                        self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "STAY"
                    
                else:
                    #print('agv路径还没走完')
                    #agv路径还没走完
                    next_loc = list(self.agvs_paths[agv_id][1])  #()=>[]
                    if tuple(next_loc) in self.obstacles: #[]=>()
                        #agv不动,stay
                        self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "STAY"
                        self.agvs[agv_id].loc = self.agvs[agv_id].loc
                        continue
                    agv_loc = self.agvs[agv_id].loc
                    self.obstacles.remove(tuple(agv_loc)) #更新障碍物
                    self.agvs[agv_id].loc = next_loc #前进一步
                    self.agvs_paths[agv_id] = self.agvs_paths[agv_id][1:] #更新路径
                    self.obstacles.add(tuple(next_loc))
                    is_loc = False #非死锁情况
                    

                    self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "MOVE"
                    #移动方向
                    direct = ""
                    if next_loc[0]-agv_loc[0] != 0:
                        if next_loc[0]-agv_loc[0] == 1:
                            direct = "DOWN"
                        else:
                            direct = "UP"
                    elif next_loc[1] - agv_loc[1] != 0:
                        if next_loc[1]-agv_loc[1] == 1:
                            direct = "RIGHT"
                        else:
                            direct = "LEFT"
                    self.ACTIONS_SEQ1[action_idx][agv_id]["dir"] = direct
                    
            return is_loc
                
                
        '''AGV运输情况'''
        def onload(self):
            agv_id = 0
            cargo_id = 0
            while agv_id<len(self.agvs) and cargo_id<len(self.cargos):
                if self.agvs[agv_id].payload != -1 or self.agvs[agv_id].occupy != -1: #当前AVG在装载或是正在去装载的路上
                    #if self.agvs_paths[agv_id] != []:      
                    agv_id += 1
                    continue
                if self.shelfs[self.cargos[cargo_id].target].payload == cargo_id or self.cargos[cargo_id].load != -1:#货物已经在货架上了或是正在被AVG前往装载
                    cargo_id += 1
                    continue
                start = self.agvs[agv_id].loc
                end = self.cargos[cargo_id].loc
                # print(agv_id, start)
                # print(cargo_id, end)
                #寻找可行的一个货物周围的位置，从而运输 
                for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                    #print(self.path_obstacles)
                    
                    path = self.bfs(start, [end[0]+dx, end[1]+dy]) 
                    if path != []: #找到了货物周围的一条可行线路
                        end = [end[0]+dx, end[1]+dy]
                        break
                self.agvs_paths[agv_id] = path #保存对应AGV的运输线路[(1,1), (2,2),...]
                if path == []:
                    #print("path为空")
                    agv_id += 1
                    continue   #无可行路径，不予分配agv和cargo
                #正常分配agv给cargo
                self.path_obstacles = self.path_obstacles.union(set(path)) #将该路径的所有经过的点都放入障碍物中
                self.agvs[agv_id].occupy = cargo_id
                self.cargos[cargo_id].load = agv_id
                print("AVG"+str(agv_id) + "运输" + str(cargo_id) +"货物")
                
                agv_id += 1
                cargo_id += 1


        '''AGV'''
        def delivery(self):
            agv_id = 0
            while agv_id<len(self.agvs):
                if self.agvs[agv_id].payload == -1 or (self.agvs[agv_id].payload != -1 and self.agvs_paths[agv_id] != []): #当前AVG没在运输,或者是当前agv在运输中，但是已经分配了路径
                    agv_id += 1
                    continue
                start = self.agvs[agv_id].loc
                cargo_id = self.agvs[agv_id].payload
                end = self.shelfs[self.cargos[cargo_id].target].loc
                #寻找可行的一个货物周围的位置，从而运输 
                for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]: 
                    
                    path = self.bfs(start, [end[0]+dx, end[1]+dy])
                    if path != []: #找到了货物周围的一条可行线路
                        end = [end[0]+dx, end[1]+dy]
                        break
                if path == []: #没有可行路径，暂时停止
                    agv_id += 1
                    continue
                self.path_obstacles = self.path_obstacles.union(set(path)) #将该路径的所有经过的点都放入障碍物中
                self.agvs_paths[agv_id] = path #保存对应AGV的运输线路    
                agv_id += 1
        
        def isDeliver(self): #判断一次送货/卸货是否完毕
            for agv_id in self.agvs_paths:
                if len(self.agvs_paths[agv_id]) != 0:
                    return False
            return True
        

        '''随机运动，不装货不卸货'''
        def random_foward(self):
            action_idx = len(self.ACTIONS_SEQ1)
            self.ACTIONS_SEQ1.append([])
            for i in range(len(self.agvs)): #本轮所有agv的操作字典
                self.ACTIONS_SEQ1[action_idx].append({})
            agvs_ids_list = [i for i in range(len(self.agvs))]
            #random.shuffle(agvs_ids_list) #AGV运输顺序为随机
            for agv_id in agvs_ids_list:
                if self.agvs_paths[agv_id] == []:
                    #print('路径为空')
                    #agvs没有要行走的路径不动
                    self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "STAY"
                    self.agvs_paths[agv_id]=[]
                    continue
                    
                elif self.agvs_paths[agv_id][0] == self.agvs_paths[agv_id][-1]: #AGV到达了终点
                    #print('agv到达终点')
                    self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "STAY"
                    self.agvs_paths[agv_id] = [] #路径清空
                        
                    
                else:
                    #print('agv路径还没走完')
                    #agv路径还没走完
                    next_loc = list(self.agvs_paths[agv_id][1])  #()=>[]
                    if tuple(next_loc) in self.obstacles: #[]=>()
                        #agv不动,stay
                        self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "STAY"
                        self.agvs[agv_id].loc = self.agvs[agv_id].loc
                        continue
                    agv_loc = self.agvs[agv_id].loc
                    self.obstacles.remove(tuple(agv_loc)) #更新障碍物
                    self.agvs[agv_id].loc = next_loc #前进一步
                    self.agvs_paths[agv_id] = self.agvs_paths[agv_id][1:] #更新路径
                    self.obstacles.add(tuple(next_loc))
                    is_loc = False #非死锁情况
                    

                    self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "MOVE"
                    #移动方向
                    direct = ""
                    if next_loc[0]-agv_loc[0] != 0:
                        if next_loc[0]-agv_loc[0] == 1:
                            direct = "DOWN"
                        else:
                            direct = "UP"
                    elif next_loc[1] - agv_loc[1] != 0:
                        if next_loc[1]-agv_loc[1] == 1:
                            direct = "RIGHT"
                        else:
                            direct = "LEFT"
                    self.ACTIONS_SEQ1[action_idx][agv_id]["dir"] = direct
            
        '''给已经装载了货物的重新规划路径'''
        def random_deliver_and_onload(self):
            for agv_id in range(len(self.agvs)):
                if self.agvs[agv_id].payload != -1:
                    start = self.agvs[agv_id].loc
                    end = self.shelfs[self.cargos[self.agvs[agv_id].payload].target].loc

                    for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:

                        path = self.bfs(start, [end[0]+dx, end[1]+dy]) 
                        if path != []: #找到了货物周围的一条可行线路
                            end = [end[0]+dx, end[1]+dy]
                            break
                    self.agvs_paths[agv_id] = path #保存对应AGV的运输线路[(1,1), (2,2),...]
                    if path == []:
                        continue   #无可行路径，不予分配agv和cargo
                    #正常分配agv给cargo
                    self.path_obstacles = self.path_obstacles.union(set(path)) #将该路径的所有经过的点都放入障碍物中
                
                if self.agvs[agv_id].payload == -1 and self.agvs[agv_id].occupy != -1:
                    '''消除货物与AGV（前去运输）的关系'''
                    self.cargos[self.agvs[agv_id].occupy].load = -1 
                    self.agvs[agv_id].occupy = -1
                    
        
        def random_move(self):
            for i in self.agvs_paths:
                ranx = random.randint(0, self.map_height)
                rany = random.randint(0, self.map_width)
                while (ranx, rany) in self.obstacles:
                    ranx = random.randint(0, self.map_height)
                    rany = random.randint(0, self.map_width)
                self.agvs_paths[i] = self.bfs(self.agvs[i].loc ,[ranx, rany])
            for i in range(10):
                self.random_foward()
            self.paths_init()
            self.path_obstacles = set(self.obstacles)
            self.random_deliver_and_onload()
            

        def process(self):
            self.paths_init()
            step = 1000
            while not self.isComplete() and step>=0:
                step-=1
                
                self.path_obstacles = set(self.obstacles)
                #一次送货卸货
                self.onload() 
                #print(self.agvs_paths)
                while not self.isDeliver():
                    locked = self.forward()
                    #print('运输中')
                    if locked:
                        print("死锁")
                        
                self.path_obstacles = set(self.obstacles)
                #print("运输完成")
                self.delivery()
                #print("正在前往装货")
                #print(self.agvs_paths)
                while not self.isDeliver():
                    #print("运货中")
                    locked = self.forward()
                    if locked:
                        print("死锁")
            if self.isComplete():
                print("全部运输完成")
            else:
                print("没有运输成功") 

                for i in range(len(self.cargos)):
                    if self.shelfs[self.cargos[i].target].payload != self.cargos[i].id:
                        print("货物"+str(self.cargos[i].id) +"没有被运输到货架上")
                
                
                self.random_move() #没有找到路线，死锁则重新随机运动寻找路径
                step=1000
                while not self.isComplete() and step>=0:
                    step-=1
                    lock_onload = False
                    lock_deliver = False
                     
                    self.path_obstacles = set(self.obstacles)
                    #一次送货卸货
                    self.onload()
                    
                    for i in self.agvs_paths:
                        print(self.agvs_paths[i])
                    print("开始新的运送")
                    
                    if self.isDeliver():
                        lock_onload = True
                        
                        
                    while not self.isDeliver():
                        locked = self.forward()
                        print('\n')
                        #print('运输中')
                        if locked:
                            print("死锁")
                            
                    self.path_obstacles = set(self.obstacles)
                    #print("运输完成")
                    self.delivery()
                    #print("正在前往装货")
                    #print(self.agvs_paths)
                    
                    if self.isDeliver():
                        lock_deliver = True
                        
                    while not self.isDeliver():
                        #print("运货中")
                        locked = self.forward()
                        if locked:
                            print("死锁")
                    if lock_onload and lock_deliver:
                        self.random_move() #没有找到路线，死锁则重新随机运动寻找路径
                    
                if self.isComplete():
                    print("全部运输完成")   
                
                
    return sf(map_info)
            


            
ACTIONS_SEQ = []
        


if __name__ == "__main__":

    resp = requests.post(f"{SERVER_URL}/create_submission")
    assert (resp.status_code == 200)
    data = resp.json()
    print ("create_submission: ", data)
    compele_map = []

    '''执行每一张地图'''
    for map_id in data['value']['maps']:
        print(map_id)
        resp = requests.post(f"{SERVER_URL}/start",json={"map_id":map_id})
        assert (resp.status_code == 200)
        map_info = resp.json() #获取地图号为map_id地图信息
        #print (map_data)
        
        game = class_init(map_info)
        game.map_process()
        ACTIONS_SEQ = game.ACTIONS_SEQ1
        
        if game.isComplete():
            compele_map.append(map_id)
        
        print(len(ACTIONS_SEQ))
        for actions in ACTIONS_SEQ:
            try:
            #print('actions:', actions)
                resp = requests.post(f"{SERVER_URL}/step",json={"map_id": map_id,"actions": actions})
                assert (resp.status_code == 200)
                data = resp.json()
                if data["value"]["done"]:
                    print ('done:', data["value"]["done"])
                    break
            except AssertionError:
                print("assert error occur")
                break;
    
    
    print(compele_map)
    '''提交结果'''
    resp = requests.post(f"{SERVER_URL}/finish_submission",json={})
    assert (resp.status_code == 200)
    data = resp.json()
    print (data)





