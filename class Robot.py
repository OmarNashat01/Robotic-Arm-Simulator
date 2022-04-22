import math
from matplotlib import pyplot


class ThreeLinkRobotRRR:
   
    def __init__(self, l1, l2, l3, th1min, th1max,th2min, th2max,th3min, th3max,):
        self.L1, self.L2, self.L3 = l1, l2, l3
        
        self.theta1min = th1min
        self.theta1max = th1max

        self.theta2min = th2min
        self.theta2max = th2max

        self.theta3min = th3min
        self.theta3max = th3max

###############################################################################################################################################

    def Dkpm(self,th1, th2, th3):
        if (th1 > self.theta1max or th1 < self.theta1min or th2 > self.theta2max or th2 < self.theta2min or th3 > self.theta3max or th3 < self.theta3min):
            return False
        
        x = self.L1 * math.cos(math.radians(th1)) + self.L2 * math.cos(math.radians(th1 + th2)) + self.L3 * math.cos(math.radians(th1 + th2 + th3))
        y = self.L1 * math.sin(math.radians(th1)) + self.L2 * math.sin(math.radians(th1 + th2)) + self.L3 * math.sin(math.radians(th1 + th2 + th3))

        return x,y
    
#####################################################################################################################################################

    def Ikpm(self,x,y):

        for theta1 in range(self.theta1min,self.theta1max+1):
            x1 = x - (self.L1 * math.cos(math.radians(theta1)))
            y1 = y - (self.L1 * math.sin(math.radians(theta1)))

            if (( ( (math.pow(x1,2) + math.pow(y1,2) - math.pow(self.L3,2) - math.pow(self.L2,2)) / (2 * self.L2 * self.L3) ) > 1 or
                ( ( (math.pow(x1,2) + math.pow(y1,2) - math.pow(self.L3,2) - math.pow(self.L2,2)) / (2 * self.L2 * self.L3) ) < -1))):
                continue
            theta3 = math.acos( (math.pow(x1,2) + math.pow(y1,2) - math.pow(self.L3,2) - math.pow(self.L2,2)) / (2 * self.L2 * self.L3) )

            theta2 = math.atan2( ( (-self.L3 * math.sin(theta3))*x1 + ( self.L2 + (self.L3 * math.cos(theta3)))*y1 ) / ((x1*x1) + (y1*y1)),
                                 ( (self.L2 + (self.L3 * math.cos(theta3)))*x1 + (self.L3 * math.sin(theta3))*y1 ) / ((x1*x1) + (y1*y1)))

            theta2 = math.degrees(theta2)
            theta2 = (theta2 - theta1) % 360
            theta3 = math.degrees(theta3)

            if (theta2 >= self.theta2min and theta2 <= self.theta2max and
                theta1 >= self.theta1min and theta1 <= self.theta1max and
                theta3 >= self.theta3min and theta3 <= self.theta3max):
                return theta1,theta2,theta3 
  
        return False 

########################################################################################################################################
    def PlotWorkingArea(self):
        X = []
        Y = []

        for Q1 in range(self.theta1min,self.theta1max,2):
            for Q2 in range(self.theta2min,self.theta2max,2):
                for Q3 in range(self.theta3min,self.theta3max,2):
                    x,y = self.Dkpm(Q1,Q2,Q3)
                    X.append(x)
                    Y.append(y)
        pyplot.ion()
        pyplot.scatter(X,Y)
        pyplot.show()
        pyplot.draw()
        pyplot.pause(3) 

    def WorkingArea(self):
        X = []
        Y = []

        for Q1 in range(self.theta1min,self.theta1max):
            x,y = self.Dkpm(Q1,self.theta2min,self.theta3min)
            X.append(x)
            Y.append(y)


        for Q2 in range(self.theta2min, self.theta2max):
            x,y = self.Dkpm(self.theta1max, Q2, self.theta3min)
            X.append(x)
            Y.append(y)

        for Q3 in range(self.theta3min, self.theta3max):
            x,y = self.Dkpm(self.theta1min, self.theta2max, Q3)
            X.append(x)
            Y.append(y)

        for Q2 in range(self.theta2max, self.theta2min,-1):
            x,y = self.Dkpm(self.theta1min, Q2, self.theta3max)
            X.append(x)
            Y.append(y)

        for Q3 in range(self.theta3max, self.theta3min,-1):
            x,y = self.Dkpm(self.theta1min, self.theta2min, Q3)
            X.append(x)
            Y.append(y)
        area = 0
        for i in range(len(X)-1):
            area += ((X[i + 1] + X[i]) * (Y[i+1] - Y[i]))/2


        return area

####################################################################################################################################################

    def StraightLineTrajectory(self, X1, Y1, X2, Y2):
        #Equation of straight line is Y = m*X + c

        if not (self.Ikpm(X1,Y1)) or (not(self.Ikpm(X2,Y2))): 
            print("this is out of robots range")
            return

        if (X2 - X1) != 0:
            step = (X2 - X1) / 100
            m = (Y2 - Y1) / (X2 - X1)
            c = Y1 - m * X1
        else:
            step = (Y2 - Y1) / 100 

        pyplot.ion()
        pyplot.show()

        for frame in range(100):
            if (X2- X1) != 0:
                X = X1 + step * frame
                Y = m * X + c
            else:
                X = X1
                Y = Y1 + step * frame
            if not (self.Ikpm(X,Y)): 
                print("this is out of robots range")
                return
            th1,th2,th3 = self.Ikpm(X,Y)

            Xl1 = [0, self.L1*math.cos(math.radians(th1))]
            Yl1 = [0, self.L1*math.sin(math.radians(th1))]

            Xl2 = [Xl1[1], Xl1[1] + self.L2*math.cos(math.radians(th1 + th2))]
            Yl2 = [Yl1[1], Yl1[1] + self.L2*math.sin(math.radians(th1 + th2))]

            Xl3 = [Xl2[1], Xl2[1] + self.L3*math.cos(math.radians(th1 + th2 + th3))]
            Yl3 = [Yl2[1], Yl2[1] + self.L3*math.sin(math.radians(th1 + th2 + th3))]

            lineX = [X1, Xl3[1]]
            lineY = [Y1, Yl3[1]]

            pyplot.clf()

            pyplot.xlim([-self.L1-self.L2-self.L3,self.L1+self.L2+self.L3])
            pyplot.ylim([-self.L1-self.L2-self.L3,self.L1+self.L2+self.L3])


            pyplot.grid()
            pyplot.scatter([0, X1, Xl3[1]], [0, Y1, Yl3[1]])
            pyplot.plot(lineX,lineY,'--', linewidth=1)
            pyplot.plot(Xl1,Yl1, linewidth=2.5)
            pyplot.plot(Xl2,Yl2, linewidth=2.5)
            pyplot.plot(Xl3,Yl3, linewidth=2.5)

            pyplot.draw()
            pyplot.pause(0.001) 
        pyplot.pause(1)


    def RobotAnimation(self):
       self.StraightLineTrajectory(self.L1,self.L2,-self.L1 + self.L3,self.L3)



print("\n\n")
print("Three Link Robot Arm (RRR) simulator....\n")

l1= float(input("Please enter the length of the 3 link of the robot: "))
l2= float(input())
l3= float(input())

theta1min = int(input("Please enter theta 1 minimum value: "))
theta1max = int(input("Please enter theta 1 maximum value: "))
theta2min = int(input("Please enter theta 2 minimum value: "))
theta2max = int(input("Please enter theta 2 maximum value: "))
theta3min = int(input("Please enter theta 3 minimum value: "))
theta3max = int(input("Please enter theta 3 maximum value: "))


Robot = ThreeLinkRobotRRR(l1,l2,l3,theta1min,theta1max,theta2min,theta2max,theta3min,theta3max)
print()
choice = int(input("Choose 1: for Dkpm\nChoose 2: for Ikpm\nChoose 3: for Area calculation\nChoose 4: for Plotting working area\nChoose 5: for generation of straigth line trajectory\nChoose 6: for Robot animation\nChose anything else to quit the program\nChoice = "))
print()
while (choice in [1,2,3,4,5,6]):

    if choice == 1:
        print("Dkpm...\n")
        theta1 = int(input("Please enter theta 1: "))
        theta2 = int(input("Please enter theta 2: "))
        theta3 = int(input("Please enter theta 3: "))

        print(f"Your X and Y coordinates are as follow: {Robot.Dkpm(( theta1%360), (theta2%360), (theta3%360) )}")

    elif choice == 2:
        print("Ikpm...\n")
        x = int(input("Please enter X: "))
        y = int(input("Please enter Y: "))
        if Robot.Ikpm(x,y):
            print(f"Your theta 1 , theta 2 theta 3 are as follow: {Robot.Ikpm(x,y)}")
        else:
            print("The coordinates you entered are out of range.")

    elif choice == 3:

        print(f"The working area of the robot is equal to: {Robot.WorkingArea()}")
     
    elif choice == 4:
        Robot.PlotWorkingArea()

    
    elif choice == 5:
        x1 = int(input("Please enter X1: "))
        y1= int(input("Please enter Y1: "))
        x2 = int(input("Please enter X2: "))
        y2 = int(input("Please enter Y2: "))
        Robot.StraightLineTrajectory(x1,y1,x2,y2)
    
    elif choice == 6:
        Robot.RobotAnimation()


    print()
    choice = int(input("Choose 1: for Dkpm\nChoose 2: for Ikpm\nChoose 3: for Area calculation\nChoose 4: for Plotting working area\nChoose 5: for generation of straigth line trajectory\nChoose 6: for Robot animation\nChose anything else to quit the program\nChoice = "))
    print()


print("Thanks for using our simulator :)\nGoodbye...\nThree Link Robot Arm (RRR) Simulator\n")