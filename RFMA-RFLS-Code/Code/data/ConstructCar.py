import json


if __name__=='__main__':
    print("Hello World")
    for j in range(99, 101):
        carfilepath = './VehicleData_'+str(j)+'.json'
        depotfilepath = './Depots.json'
        NUM = j

        tmp = set()
        count = 0

        carList = []
        depotList = []
        t = 0
        depotInd = 0

        f = open(carfilepath, 'w', encoding='utf-8')
        d = open(depotfilepath, 'r', encoding='utf-8')

        depots = json.load(d)
        # print(depots)

        d.close()

        for i in depots["depot"]:
            depotList.append(i["id"])

        while count < NUM:
            s = hash(str(t))
            t += 1
            if s not in tmp:
                tmp.add(s)
                carList.append(
                    {"currentCapacity": 0, "id": 'sustech_'+str(s), "maxCapacity": 720, "no": str(t), "stopId": depotList[depotInd]})
                depotInd = (depotInd + 1) % len(depotList)
                count += 1
        car_dict = {"vehicle":carList}
        json.dump(car_dict, f)
        f.close()
