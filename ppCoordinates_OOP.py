import math
from math import sqrt
from numpy import *
import time
import pdb

class PathPlanner:
        """
        pathPlanner gets data from position estimator and worldModelClient via queue_pathPlanner
        path is sent to tracker via queue_tracker
        """
        def __init__(self):
                self.X=[]#x coordinates of obstacles, list of lists
                self.Y=[]#y coordinates of obstacles, list of lists
                self.X1=[]#X1,Y1 are just list of X,Y respect., not list of lists
                self.Y1=[]
                self.Xovlap=[]#x-coordinate of overlapping point
                self.Yovlap=[]#y-coordinate of overlapping point
                self.OVptID=[]#Overlapped point ID
                self.FS=[]#For 'False sides' idenfitication
                self.objno=0
                self.ptvis1=[]#all visible lines
                self.ptvis2=[]
                self.ptvis3=[]#visible lines excluded overlapping points or sides
                self.ptvis4=[]
                self.Xs=0.0 #range of world model
                self.Xe=20.0
                self.Ys=0.0
                self.Ye=20.0
                self.XY=[]#"""point wise list of list including start and end"""


        def run(self):  
                #obstacleList, vehicleList, chessList, wmTime = receivedData    #wmTime is time, when DX Request was sent from WM
                obstacleList=[[3.25,1.2,0,0,3,2,120,0],[1.25,8.0,0,0,2,3,100,1],
                         [7.8,8.5,0,0,2.5,2.5,90,2],[8.8,1.5,0,0,4.2,2.4,80,3],
                         [5.0,5.5,0,0,3.2,2.3,70,4],[4.25,11.2,0,0,2.6,2.9,60,5],
                         [10.8,4.9,0,0,3.4,2.4,40,6],[3.25,16,0,0,3.1,1.3,30,7],
                         [16.25,3.5,0,0,3.7,2.7,20,8],[8.0,15.5,0,0,2.3,4.0,10,9],
                         [11,12.5,0,0,3,3.8,0,10],[11,8,0,0,1.9,2.9,-10,11],
                         [16.5,17.5,0,0,2.9,2.1,-20,12],[18.0,10.2,0,0,1.5,3.5,-30,13]]
                vehicleList=[[0.0,0.0,10,60]]
                chessList=[[20.0,20.0,20,65]]

                # obstacleList: [[x, y, x_p, y_p, a, b, phi, id],...]
                # vehicleList: [[x, y, phi, id],...]
                # chessList: [[x, y, phi, id],...]
                # 
                # ---------------------------------> x
                # |
                # |
                # |
                # |
                # v  y
                # 
                
                #object coordinates X,Y; list of lists
                self.X,self.Y=self.objcoordinates(obstacleList)
                self.objno=len(self.X)                            
                start=self.objno*4
                end=self.objno*4+1
                stcoordinate=[vehicleList[0][0],vehicleList[0][1]]
                dstcoordinate=[chessList[0][0],chessList[0][1]]
                self.XYtoX1Y1()
                self.sortingX1Y1()
                self.identifyOverlap()        
                self.connectingLines()
                self.XYtoX1Y1()
                self.visibleLinesFromStartEndPoint(stcoordinate,dstcoordinate )
                self.visibleLinesAmongObjects()
                self.exclusivelyVisibleLines()
                Graph=self.graphConstruction(start,end)
                self.XYconstruct(stcoordinate,dstcoordinate)
                MD=self.astar_shortest_path(Graph, start, end)
                MDcoordinate=self.MinpathCoordinate(MD)
                print(MDcoordinate)
                                        
        def objcoordinates(self,obslist):
                
                """######calculating coordinates of obstacles"""
                x=[]
                y=[]
                dx=dy=0.3
                for L in obslist:
                        phi=2.0*math.pi - math.radians(L[6])
                        x1=L[0] + math.hypot(dx+L[4]/2.0,dy+L[5]/2.0)*math.cos(phi)
                        y1=L[1] + math.hypot(dx+L[4]/2.0,dy+L[5]/2.0)*math.sin(phi)
                        x2=L[0] + math.hypot(dx+L[4]/2.0,dy+L[5]/2.0)*math.cos(phi+math.pi/2.0)
                        y2=L[1] + math.hypot(dx+L[4]/2.0,dy+L[5]/2.0)*math.sin(phi+math.pi/2.0)
                        x3=L[0] + math.hypot(dx+L[4]/2.0,dy+L[5]/2.0)*math.cos(phi+math.pi)
                        y3=L[1] + math.hypot(dx+L[4]/2.0,dy+L[5]/2.0)*math.sin(phi+math.pi)
                        x4=L[0] + math.hypot(dx+L[4]/2.0,dy+L[5]/2.0)*math.cos(phi+3.0*math.pi/2.0)
                        y4=L[1] + math.hypot(dx+L[4]/2.0,dy+L[5]/2.0)*math.sin(phi+3.0*math.pi/2.0)
                        x.append([x1,x2,x3,x4])
                        y.append([y1,y2,y3,y4])
                return x,y
        
        def XYtoX1Y1(self):
                """From list of lists X,Y to simple list X1,Y1"""
                self.X1=[]
                self.Y1=[]
                for g,h in zip(self.X,self.Y):
                        self.X1=self.X1+g
                        self.Y1=self.Y1+h
                #print len(self.X1),len(self.Y1)
        def sortingX1Y1(self):
                """Sorting X1 for sweep line, and Y1 follows X1"""
                for i in range(len(self.X1)-1):
                        for j in range(len(self.X1)-1-i):
                                if(self.X1[j]>self.X1[j+1]):
                                        tempx=self.X1[j]
                                        self.X1[j]=self.X1[j+1]
                                        self.X1[j+1]=tempx
                                        tempy=self.Y1[j]
                                        self.Y1[j]=self.Y1[j+1]
                                        self.Y1[j+1]=tempy
                """Now X1 is ascending order sorted list of x coordinates of obstacles, Y1 follows X1"""
                                        
        def ID(self,a,b):
                """point (a,b) belongs to I'th object position of X,Y"""
                for I in range(len(self.X)):
                        got=0
                        for J in range(len(self.X[I])):
                            if(a==self.X[I][J] and b==self.Y[I][J]):
                                got=1
                                break
                        if(got==1):
                            break
                return I
        
        def Xingpoint(self,a,b,c,d,e,f,g,h):
            """ 1stLine=(a,b),(c,d); 2ndLine=(e,f),(g,h)"""
            #########Find crossing point of two lines##########################
            if ((a-c)==0) and ((e-g)==0):
                crx=cry=-20.3456 #an arbitrary value to avoid if at the later part
            elif  (b-d)==0 and (e-g)==0:
                crx=e
                cry=b
            elif  (f-h)==0 and (a-c)==0 :
                crx=a
                cry=f
            elif  ((e-g)==0) and (a!=c) and (b!=d):
                th1=(b-d)/(a-c)
                crx=e
                cry=b+th1*(e-a)
            elif (a-c)==0 and e!=g:
                th2=(f-h)/(e-g)
                crx=a
                cry=f+th2*(a-e)
                    
            elif (b-d)/(a-c)== (f-h)/(e-g):#better to use assert  
                crx=cry=-20.3456 #an arbitrary value to avoid if at the later part
            elif  (b-d)==0 and (e!=g) :#better to use assert 
                cry=b
                th2=(f-h)/(e-g)
                crx=e+(cry-f)/th2
            else:
                th1=(b-d)/(a-c)
                th2=(f-h)/(e-g)
                cry= (th1*f + th1*th2*(a-e) - th2*b)/(th1-th2) #cry is crossing point y
                crx= a + (cry-b)/th1
            return crx,cry
        """##################################################################################"""
        
        def identifyOverlap(self):
            """#######Overlapping points and sides#############################################"""
            for k in range(len(self.X1)): #point
                objID=self.ID(self.X1[k],self.Y1[k])
                for l in range(len(self.X)): #another object
                    if(l!=objID):
                        if((self.X1[k]>=min(self.X[l]) and self.X1[k]<=max(self.X[l])) and (self.Y1[k]>=min(self.Y[l]) and self.Y1[k]<=max(self.Y[l]))):
                            Xys=[] #Xx=x coordinate of crossing, and hence Xy, Xys=all crossing y-coordinate
                            for m in range(len(self.X[l])):
                                if(m==len(self.X[l])-1):
                                    M=0
                                else:
                                    M=m+1
                                if(self.X[l][m]!=self.X[l][M]):
                                    Xx=self.X1[k]
                                    Xy=self.Y[l][m]+ (self.X1[k]-self.X[l][m])*(self.Y[l][m]-self.Y[l][M])/(self.X[l][m]-self.X[l][M])
                                    if ((Xx-self.X[l][m])*(Xx-self.X[l][M]) + (Xy-self.Y[l][m])*(Xy-self.Y[l][M]))<0:
                                         Xys.append(Xy)
                            if(self.Y1[k]<=max(Xys) and self.Y1[k]>=min(Xys)):
                                    self.Xovlap.append(self.X1[k])
                                    self.Yovlap.append(self.Y1[k])
                    
                                    for L in range(len(self.X[objID])): #For determining the next point of X1[k],Y1[k]
                                        if self.X1[k]==self.X[objID][L] and self.Y1[k]==self.Y[objID][L]:
                                            self.OVptID.append(objID*4+L)
                                            if L==len(self.X[objID])-1:
                                                XO=self.X[objID][0] ##Next point of overlapped corner point
                                                YO=self.Y[objID][0]
                                            else:
                                                XO=self.X[objID][L+1]
                                                YO=self.Y[objID][L+1]
                                    #pdb.set_trace() 
                                    for m in range(len(self.X[l])):#have to consider many case for intersection
                                        if(m==len(self.X[l])-1):
                                            M=0
                                        else:
                                            M=m+1
                                        crx,cry=self.Xingpoint(self.X1[k],self.Y1[k],XO,YO,self.X[l][m],self.Y[l][m],self.X[l][M],self.Y[l][M])
                                        if crx!=-20.3456 and cry!=-20.3456:
                                            crva= [crx-self.X1[k], cry-self.Y1[k] ]   #vector from crossing point to a
                                            crvb= [crx-XO, cry-YO ]
                                            crvc= [crx-self.X[l][m], cry-self.Y[l][m] ]   #vector from crossing point to c
                                            crvd= [crx-self.X[l][M], cry-self.Y[l][M] ]
                                        if (crva[0]*crvb[0] + crva[1]*crvb[1])<0 and (crvc[0]*crvd[0] + crvc[1]*crvd[1])<0:
                                            NI=self.ID(self.X[l][m],self.Y[l][m])
                                            self.FS.append([4*NI+m,4*NI+M] )

        def connectingLines(self):
                """data of connecting lines within the objects####################"""
                objno=len(self.X)
                self.ptvis1=list(range(objno*4))
                self.ptvis2=list(range(1,(objno*4)+1))
                for A in self.ptvis2:
                    ind=self.ptvis2.index(A)
                    if ((ind+1)%4==0):
                        self.ptvis2.remove(A)
                        self.ptvis2.insert(ind,self.ptvis1[ind-3])
                #return ptvis1,ptvis2

        def visibleornot(self,a,b,x,y):#only valid for rectangualr object
                """point a and b is visible or not"""
                for N in range(self.objno*2):
                        if N%2==0:
                                c=2*N
                                d=c+2
                        else:
                                c=2*N-1
                                d=c+2

                        if(a!=c and a!=d and b!=c and b!=d):
                            crx,cry=self.Xingpoint(x[a],y[a],x[b],y[b],x[c],y[c],x[d],y[d])
                            if crx!=-20.3456 and cry!=-20.3456:
                                crva= [crx-x[a], cry-y[a] ]   #vector from crossing point to a
                                crvb= [crx-x[b], cry-y[b] ]
                                crvc= [crx-x[c], cry-y[c] ]   #vector from crossing point to c
                                crvd= [crx-x[d], cry-y[d] ]
                                if (crva[0]*crvb[0] + crva[1]*crvb[1])<=0 and (crvc[0]*crvd[0] + crvc[1]*crvd[1])<=0:
                                    draw=0
                                    break
                                else:
                                    draw=1
                            else:
                                draw=1
                return draw
        

        def visibleLinesFromStartEndPoint(self,stc,dstc ):
                """##############visibility lines drawing from start and end points"""
                self.X1.append(stc[0])
                self.Y1.append(stc[1])
                self.X1.append(dstc[0])
                self.Y1.append(dstc[1])
                for S in range(len(stc)):
                    SE=self.objno*4+S
    
                    for seo in range(SE):
                        if SE!=seo and self.X1[seo] not in self.Xovlap and self.Y1[seo] not in self.Yovlap:
                            draw=self.visibleornot(SE,seo,self.X1,self.Y1)
                            if draw==1:
                                if S==0:
                                    #plt.plot([X1[SE],X1[seo]],[Y1[SE],Y1[seo]], 'y')#working fine
                                    self.ptvis1.append(SE) #saving first point of visible line 
                                    self.ptvis2.append(seo) #saving 2nd point of visible line
                                else:
                                    self.ptvis1.append(seo) #saving first point of visible line 
                                    self.ptvis2.append(SE)
                self.X1.pop()
                self.X1.pop()
                self.Y1.pop()
                self.Y1.pop()

        def visibleLinesAmongObjects(self):
                """ visibility checking and drawing among objects"""
                for i in range(self.objno-1):                #i for object
                   for k in range(len(self.X[i])):           #k for points of an object
                        for j in range(len(self.X1)-4*(i+1)):        #j for points of other objects
                          # a, b are two points to which line to be drawn or not, while, c,d are nearest two points of a
                            a= 4*i+k
                            b= j+4+4*i
                            #print len(self.X1)
                            #pdb.set_trace()
                            if ((self.X[int(a/4)][a%4]  in self.Xovlap) and (self.Y[int(a/4)][a%4]  in self.Yovlap)) or ((self.X[int(b/4)][b%4] in self.Xovlap) and (self.Y[int(b/4)][b%4]  in self.Yovlap)):
                                draw=0 #pass #break
                            else:
                                draw=self.visibleornot(a,b,self.X1,self.Y1)
                            if draw==1 :
                                self.ptvis1.append(a) #saving first point of visible line 
                                self.ptvis2.append(b) #saving 2nd point of visible line

        def exclusivelyVisibleLines(self):
                """#####Removing visible points outside the data world#########"""
                """###copying exclusively visible paths to ptvis3 and ptvis4"""
                Idxy=[]
                for p,q in zip(self.X1,self.Y1):
                    #print p,q
                    
                    if p<self.Xs or p>self.Xe:
                        Idxy.append(self.X1.index(p))
                    elif q<self.Ys or q>self.Ye:
                        Idxy.append(self.Y1.index(q))
                    else:
                        pass
                for ex,ey in zip(self.ptvis1,self.ptvis2):
                    if (ex not in Idxy) and (ey not in Idxy):
                        if ([ex,ey] not in self.FS) and ([ey,ex] not in self.FS):
                            if (ex not in self.OVptID) and (ey not in self.OVptID):
                                self.ptvis3.append(ex)
                                self.ptvis4.append(ey)    

        def graphConstruction(self,start,end):
                """ constructing graph(dictionary) from list ptvis3,4  """
                graph={}
                for r,s in zip(self.ptvis3,self.ptvis4):
                    if r==start or s==end  :
                        if r not in list(graph.keys()):
                            graph[r]=[]
                            graph[r].append(s)
                        else:
                            graph[r].append(s) 
                    elif s==start or r==end:
                        if s not in list(graph.keys()):
                            graph[s]=[]
                            graph[s].append(r)
                        else:
                            graph[s].append(r)
                    else:
                        if r not in list(graph.keys()):
                            graph[r]=[]
                            graph[r].append(s)
                        else:
                            graph[r].append(s)
                        if s not in list(graph.keys()):
                            graph[s]=[]
                            graph[s].append(r)
                        else:
                            graph[s].append(r)
    
                for N in range(end):
                    if N not in list(graph.keys()):
                        graph[N]=[]
                if (end in graph)==0:
                    graph[end]=[]
                if graph[end]!=[]:
                    for f in graph[end]:
                        graph[f].append(end)
                    graph[end]=[]
                return graph

        def XYconstruct(self, stco,dstco):
                """constructing XY from X1,Y1"""
                for R,S in zip(self.X1,self.Y1):
                    self.XY.append([R,S])
                self.XY.append(stco)
                self.XY.append(dstco)
                
        def distance(self,a,b):
                """Distance between two points a and b"""    
                return sqrt((self.XY[a][0]-self.XY[b][0])**2+(self.XY[a][1]-self.XY[b][1])**2)

        def astar_shortest_path(self,graph, start, end):
                """A* ALGORITHM"""
                OL=[] #open list
                CL=[] #close list
                minpath=[]
                i=0
                cn=start #cn=current node
                #pdb.set_trace()
                while(i<=1000): #say, maximum 100 nodes
                        if cn==end:
                            return minpath
                            break
                        #condition if path not found
                        elif (OL==[]) and (graph[cn]==[]):
                            minpath=[]
                            return minpath
                            break
        
                        elif(cn==start):
                            h=self.distance(cn, end)
                            g=0
                            f=g+h
                            CL.append(cn)
                            for v in range(len(graph[cn])):
                                if (v==0):
                                    minpath=[(self.distance(cn, graph[cn][v]) + self.distance( graph[cn][v], end)),cn, graph[cn][v]]
                                else:
                                    if (minpath[0]> (self.distance(cn, graph[cn][v]) + self.distance( graph[cn][v], end))):
                                        OL.append(minpath)
                                        minpath=[(self.distance(cn, graph[cn][v]) + self.distance( graph[cn][v], end)),cn, graph[cn][v]]
                                    else:
                                        OL.append([self.distance(cn, graph[cn][v]) + self.distance( graph[cn][v], end),cn, graph[cn][v]])
                            cn=minpath[-1]
                        else:
                            temp=cn
                            g= minpath[0]-self.distance(cn,end)
                            #OLmin search from OL
                            if (OL!=[]):
                                OLmin=OL[0]
                                for s in OL: #if OL!=NULL
                                    if(OLmin[0]>s[0]) and (s[-1] not in CL):
                                        OLmin=s
                            else:
                                OLmin=[]
                            #possible minpath finding from node
                            for v in range(len(graph[cn])):
                                f=(g+self.distance(cn, graph[cn][v]) + self.distance( graph[cn][v], end))
                                if v==0:
                                    minpath[0]=f
                                    minpath.append(graph[cn][v])
                                else:
                                    if minpath[0]>f:
                                        OL.append(minpath)
                                        minpath[0]=f
                                        minpath.pop()
                                        minpath.append(graph[cn][v])
                                    else:
                                        addOL=[]
                                        addOL=list(minpath)
                                        addOL.pop()
                                        addOL[0]=f
                                        addOL.append(graph[cn][v])
                                        OL.append(addOL)
                            ###select the nxt node wrt min f
                            if OLmin!=[]:
                                if (OLmin[0]>minpath[0]):
                                    cn=minpath[-1]
                                else:
                                    OL.append(minpath)
                                    minpath=OLmin
                                    cn=minpath[-1]
                            else:
                                cn=minpath[-1]
                            #temp is visited, so set it to CL
                            CL.append(temp)
                            #some paths are coping multiple time in OL, do delete more than one
                            #OL=list(set(OL))
                            OL1=[]
                            for element in OL:
                                if element not in OL1:
                                    OL1.append(element)
                            OL=OL1
                            #delete temp from OL
                            q=0
                            for d in OL:
                                if (d[-1]==temp):
                                    OL.pop(q)
                                q=q+1
                        #count while loop iteration
                        #print minpath
                        i=i+1 #end of while loop
        """END OF A STAR ALGORITH<<<<<<<<<<<<<<<<<<<<<<<<<"""

        def MinpathCoordinate(self, SP):
                """returning Minpath in the form of coordinates"""
                #print SP
                if (SP !=[]) and (SP != None):
                    del SP[0]
                    CoordinateSP=[]
                    for r in SP:
                        CoordinateSP.append([self.XY[r][0],self.XY[r][1]])
                else:
                        CoordinateSP=[]
                return CoordinateSP
        


PP=PathPlanner()
PP.run()

