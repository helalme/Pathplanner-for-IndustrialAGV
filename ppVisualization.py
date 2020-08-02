# Implementation of A* algorithm with procedural style, for calculating the shortest path for AGV

import pdb
import matplotlib.pylab as plt
from math import sqrt
import math 

#################################################################
###### objcoordinates() is for converting camera data into x,y coordinates
def objcoordinates(obslist):
    x=[]
    y=[]
    
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
########### END of objcoordinates() ###############################


######################################################################
####### ID(): OVERLAPPING POINTS IDENTIFICATION ######################
def ID(a,b):
    for I in range(len(X)):
        got=0
        for J in range(len(X[I])):
            if(a==X[I][J] and b==Y[I][J]):
                got=1
                break
        if(got==1):
            break
    return I
############ END of ID() ##############################################

######################################################################
########## Xingpoint(): Find crossing point of two lines ##############
def Xingpoint(a,b,c,d,e,f,g,h):# 1stLine=(a,b),(c,d); 2ndLine=(e,f),(g,h)
    if  (a-c)==0 and (e-g)==0:
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
                    
    elif (b-d)/(a-c)== (f-h)/(e-g):#assert use kora better 
        crx=cry=-20.3456 #an arbitrary value to avoid if at the later part
    elif  (b-d)==0 and (e!=g) :#assert use kora better 
        cry=b
        th2=(f-h)/(e-g)
        crx=e+(cry-f)/th2
    else:
        th1=(b-d)/(a-c)
        th2=(f-h)/(e-g)
        cry= (th1*f + th1*th2*(a-e) - th2*b)/(th1-th2) #cry is crossing point y
        crx= a + (cry-b)/th1
    return crx,cry
################# END of Xingpoint() ###############################


#####################################################################
##############visibleornot function##################################
def visibleornot(a,b,x,y,objno):#only valid for rectangular object
    for N in range(objno*2):
        if N%2==0:
            c=2*N
            d=c+2
        else:
            c=2*N-1
            d=c+2

        if(a!=c and a!=d and b!=c and b!=d):
            if  (x[a]-x[b])==0 and (x[c]-x[d])==0:
                crx=cry=-20.3456 #an arbitrary value to avoid if at the later part
            elif  (y[a]-y[b])==0 and (x[c]-x[d])==0:
                crx=x[c]
                cry=y[a]
            elif  (y[c]-y[d])==0 and (x[a]-x[b])==0 :
                crx=x[a]
                cry=y[c]
            elif  ((x[c]-x[d])==0) and (x[a]!=x[b]) and (y[a]!=y[b]):
                th1=(y[a]-y[b])/(x[a]-x[b])
                crx=x[c]
                cry=y[a]+th1*(x[c]-x[a])
            elif (x[a]-x[b])==0 and x[c]!=x[d]:
                th2=(y[c]-y[d])/(x[c]-x[d])
                crx=x[a]
                cry=y[c]+th2*(x[a]-x[c])
                    
            elif (y[a]-y[b])/(x[a]-x[b])== (y[c]-y[d])/(x[c]-x[d]):#assert use kora better 
                crx=cry=-20.3456 #an arbitrary value to avoid if at the later part
            elif  (y[a]-y[b])==0 and (x[c]!=x[d]) :#assert use kora better 
                cry=y[a]
                th2=(y[c]-y[d])/(x[c]-x[d])
                crx=x[c]+(cry-y[c])/th2
            else:
                th1=(y[a]-y[b])/(x[a]-x[b])
                th2=(y[c]-y[d])/(x[c]-x[d])
                cry= (th1*y[c] + th1*th2*(x[a]-x[c]) - th2*y[a])/(th1-th2) #cry is crossing point y
                crx= x[a] + (cry-y[a])/th1
                        
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
###########END OF visibleornot() FUNCTION #############################



################################################################
##########A* ALGORITHM##########################################
def distance(a,b):
    return sqrt((XY[a][0]-XY[b][0])**2+(XY[a][1]-XY[b][1])**2)
def astar_shortest_path(graph, xy, start, end):
    OL=[] #open list
    CL=[] #close list
    minpath=[]
    i=0
    cn=start #cn=current node
    #pdb.set_trace()
    while(i<=1000): #say, maximum 1000 nodes
        if cn==end:
            return minpath
            break
        #condition if path not found
        elif (OL==[]) and (graph[cn]==[]):
            minpath=[]
            return minpath
            break
        
        elif(cn==start):
            h=distance(cn, end)
            g=0
            f=g+h
            CL.append(cn)
            for v in range(len(graph[cn])):
                if (v==0):
                    minpath=[(distance(cn, graph[cn][v]) + distance( graph[cn][v], end)),cn, graph[cn][v]]
                    
                else:
                    
                    if (minpath[0]> (distance(cn, graph[cn][v]) + distance( graph[cn][v], end))):
                        OL.append(minpath)
                        minpath=[(distance(cn, graph[cn][v]) + distance( graph[cn][v], end)),cn, graph[cn][v]]
                        
                    else:
                        OL.append([distance(cn, graph[cn][v]) + distance( graph[cn][v], end),cn, graph[cn][v]])

            cn=minpath[-1]
        
        else:
            temp=cn
            g= minpath[0]-distance(cn,end)
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
                
                f=(g+distance(cn, graph[cn][v]) + distance( graph[cn][v], end))
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
        #print minpath
        #pdb.set_trace()
        #count while loop iteration 
        i=i+1 #end of while loop

#############################################################
########END OF A STAR ALGORITH ##############################

        
####''''''' MAIN PART start here ''''''''''''''''''#####################
## Setting data world, perception area is 20*20. Smallest unit dx=dy, and user defined e.g. 0.4
Xs=0.0
Ys=0.0
Xe=20.0
Ye=20.0

##################################################################
###Setting objects and their coordinates
#start=raw_input("Enter start point (0-19):")
#end=raw_input("Enter destination points (0-19):")
Obslist=[[3.25,1.2,0,0,3,2,120,0],[1.25,8.0,0,0,2,3,100,1],
         [7.8,8.5,0,0,2.5,2.5,90,2],[8.8,1.5,0,0,4.2,2.4,80,3],
         [5.0,5.5,0,0,3.2,2.3,70,4],[4.25,11.2,0,0,2.6,2.9,60,5],
         [10.8,4.9,0,0,3.4,2.4,40,6],[3.25,16,0,0,3.1,1.3,30,7],
         [16.25,3.5,0,0,3.7,2.7,20,8],[8.0,15.5,0,0,2.3,4.0,10,9],
         [11,12.5,0,0,3,3.8,0,10],[11,8,0,0,1.9,2.9,-10,11],
         [16.5,17.5,0,0,2.9,2.1,-20,12],[18.0,10.2,0,0,1.5,3.5,-30,13]]

objno=len(Obslist)
start=objno*4
end=objno*4+1
sdx=[0.0,20.0] # X-coordinate of start and destination points
sdy=[0.0,0.0]  # Y-coordinate of start and destination points
origin=(sdx[0],sdy[0])
destination=(sdx[1],sdy[1])
print('Start:', origin, 'and destination:', destination)
dx=dy=0.3
dxy=sqrt(2)* dx #distance between two corners


X,Y=objcoordinates(Obslist)
for b,c in zip(X,Y):
    b.append(b[0])
    c.append(c[0])
    plt.plot(b,c,'b')
    b.pop()
    c.pop()

####################################################
#####From list of lists X,Y to simple list X1,Y1
X1=[]#X1,Y1 are just list of X,Y respect., not list of lists
Y1=[]
for g,h in zip(X,Y):
    g.append(g[0])
    h.append(h[0])
    plt.plot(g,h,'r')
    g.pop()
    h.pop()
    X1=X1+g
    Y1=Y1+h

##########increased size single list X1,Y1 are being assigned to sx,sy
####################################################################
#sx=X1
#sy=Y1
####################################################################
#print X1
#print x
#print y
#print sx
#print sy
#Sorting X1 for sweep line, and Y1 follows X1
for i in range(len(X1)-1):
    for j in range(len(X1)-1-i):
        if(X1[j]>X1[j+1]):
           tempx=X1[j]
           X1[j]=X1[j+1]
           X1[j+1]=tempx
           tempy=Y1[j]
           Y1[j]=Y1[j+1]
           Y1[j+1]=tempy

##### X and Y increased unsorted list of lists, but X1,Y1 sorted
#print X
#print Y
#print X1
#print Y1


             
#for k in range(len(X1)):
#    objID=ID(X1[k],Y1[k])
#    print ID
###################################################################################
#######Overlapping points and sides#############################################
Xovlap=[]
Yovlap=[]
OVptID=[]
FS=[]#For 'False sides' idenfitication 
for k in range(len(X1)): #point
    objID=ID(X1[k],Y1[k])
    for l in range(len(X)): #another object
        if(l!=objID):
            if((X1[k]>=min(X[l]) and X1[k]<=max(X[l])) and (Y1[k]>=min(Y[l]) and Y1[k]<=max(Y[l]))):
                Xys=[] #Xx=x coordinate of crossing, and hence Xy, Xys=all crossing y-coordinate
                for m in range(len(X[l])):
                    if(m==len(X[l])-1):
                        M=0
                    else:
                        M=m+1
                    if(X[l][m]!=X[l][M]):
                        Xx=X1[k]
                        Xy=Y[l][m]+ (X1[k]-X[l][m])*(Y[l][m]-Y[l][M])/(X[l][m]-X[l][M])
                        if ((Xx-X[l][m])*(Xx-X[l][M]) + (Xy-Y[l][m])*(Xy-Y[l][M]))<0:
                            Xys.append(Xy)
                if(Y1[k]<=max(Xys) and Y1[k]>=min(Xys)):
                    Xovlap.append(X1[k])
                    Yovlap.append(Y1[k])
                    
                    for L in range(len(X[objID])): #For determining the next point of X1[k],Y1[k]
                        if X1[k]==X[objID][L] and Y1[k]==Y[objID][L]:
                            OVptID.append(objID*4+L)
                            if L==len(X[objID])-1:
                                XO=X[objID][0] ##Next point of overlapped corner point
                                YO=Y[objID][0]
                            else:
                                XO=X[objID][L+1]
                                YO=Y[objID][L+1]
                    #pdb.set_trace() 
                    for m in range(len(X[l])):#have to consider many case for intersection
                        if(m==len(X[l])-1):
                            M=0
                        else:
                            M=m+1
                        crx,cry=Xingpoint(X1[k],Y1[k],XO,YO,X[l][m],Y[l][m],X[l][M],Y[l][M])
                        if crx!=-20.3456 and cry!=-20.3456:
                            crva= [crx-X1[k], cry-Y1[k] ]   #vector from crossing point to a
                            crvb= [crx-XO, cry-YO ]
                            crvc= [crx-X[l][m], cry-Y[l][m] ]   #vector from crossing point to c
                            crvd= [crx-X[l][M], cry-Y[l][M] ]
                        if (crva[0]*crvb[0] + crva[1]*crvb[1])<0 and (crvc[0]*crvd[0] + crvc[1]*crvd[1])<0:
                            NI=ID(X[l][m],Y[l][m])
                            FS.append([4*NI+m,4*NI+M] )
                            
#print Xovlap, Yovlap
#print OVptID
#print FS
###############################################################################

#################################################
# for  points making line and construting object
ptvis1=list(range(objno*4))
ptvis2=list(range(1,(objno*4)+1))
for A in ptvis2:
    ind=ptvis2.index(A)
    if ((ind+1)%4==0):
        ptvis2.remove(A)
        ptvis2.insert(ind,ptvis1[ind-3])


############from sorted X1,Y1 to again simple lists
X1=[]#X1,Y1 are just list of X,Y respect., not list of lists
Y1=[]
for g,h in zip(X,Y):
    g.append(g[0])
    h.append(h[0])
    plt.plot(g,h,'r')
    g.pop()
    h.pop()
    X1=X1+g
    Y1=Y1+h
    
#########################################################
##############visibility lines drawing from start and end points
X1.append(sdx[0])
Y1.append(sdy[0])
X1.append(sdx[1])
Y1.append(sdy[1])
for S in range(len(sdx)):
    SE=objno*4+S
    
    for seo in range(SE):
        if SE!=seo and X1[seo] not in Xovlap and Y1[seo] not in Yovlap:
            draw=visibleornot(SE,seo,X1,Y1,objno)
            if draw==1:
                if S==0:
                    #plt.plot([X1[SE],X1[seo]],[Y1[SE],Y1[seo]], 'y')#working fine
                    ptvis1.append(SE) #saving first point of visible line 
                    ptvis2.append(seo) #saving 2nd point of visible line
                else:
                    ptvis1.append(seo) #saving first point of visible line 
                    ptvis2.append(SE)

X1.pop()
X1.pop()
Y1.pop()
Y1.pop()


###############################################
# visibility checking and drawing among objects
for i in range(objno-1):          	#i for object
    
    for k in range(len(X[i])):      	#k for points of an object
        for j in range(len(X1)-4*(i+1)):	#j for points of other objects
          # a, b are two points to which line to be drawn or not, while, c,d are nearest two points of a
            a= 4*i+k
            b= j+4+4*i
            ###################
              
            if ((X[int(a/4)][a%4]  in Xovlap) and (Y[int(a/4)][a%4]  in Yovlap)) or ((X[int(b/4)][b%4] in Xovlap) and (Y[int(b/4)][b%4]  in Yovlap)):
                draw=0 #pass #break
            else:
                draw=visibleornot(a,b,X1,Y1,objno)
                  
            if draw==1 :
                
                #plt.plot([X1[a],X1[b]],[Y1[a],Y1[b]], 'k')
                
                ptvis1.append(a) #saving first point of visible line 
                ptvis2.append(b) #saving 2nd point of visible line


#########################################################################
######Removing visible points outside the data world##########################
Idxy=[]
for p,q in zip(X1,Y1):
    #print p,q
    #pdb.set_trace()
    if p<Xs or p>Xe:
        Idxy.append(X1.index(p))
    elif q<Ys or q>Ye:
        Idxy.append(Y1.index(q))
    else:
        pass
ptvis3=[]
ptvis4=[]
for ex,ey in zip(ptvis1,ptvis2):
    if (ex not in Idxy) and (ey not in Idxy):
        if ([ex,ey] not in FS) and ([ey,ex] not in FS):
            if (ex not in OVptID) and (ey not in OVptID):
                ptvis3.append(ex)
                ptvis4.append(ey)    

#print Idxy
#print ptvis3, ptvis4


###########################################################
# constructing graph
graph={}
for r,s in zip(ptvis3,ptvis4):
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
#for g in graph.items():
#     print g



###########################################################
# constructing XY
XY=[]
for R,S in zip(X1,Y1):
    XY.append([R,S])
XY.append([sdx[0],sdy[0]])
XY.append([sdx[1],sdy[1]])
#print XY


#Shortest path includes shortest distance and points ID's 
ShortestPath = astar_shortest_path(graph, XY, start, end)
#print(ShortestPath)

if (ShortestPath !=[]) and (ShortestPath != None):
    del ShortestPath[0]
    SP=ShortestPath
    for r in range(len(SP)-1):
        plt.plot([XY[SP[r]][0],XY[SP[r+1]][0]],[XY[SP[r]][1],XY[SP[r+1]][1]], 'k')
        #print XY
         
plt.xlim(Xs,Xe)
plt.ylim(Ye,Ys)
plt.show()

