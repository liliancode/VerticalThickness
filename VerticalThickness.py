import math
import numpy as np
import matplotlib.pyplot as plt
import time
 

sleeptime = 1  # wait x sec between each iteration

Upper = [] 
Lower = []


################################################################################
#
#  geometric functions

# determine the min/max slopes of the tangent to the given point

def RSlope(LP, pos):
   dY = LP[pos+1][1]-LP[pos][1]
   dX = LP[pos+1][0]-LP[pos][0]
   return dY/dX
   
def LSlope(LP, pos):
   dY = LP[pos][1]-LP[pos-1][1]
   dX = LP[pos][0]-LP[pos-1][0]
   return dY/dX
   
def VectProd(A,B):
   return A[0]*B[1]-A[1]*B[0]
   
def VertDist(A,B,K): # vertical distance between segment AB and point K
   if ( K[0] < min(A[0],B[0]) ): return 0 # on the left  of [AB]
   if ( K[0] > max(A[0],B[0]) ): return 0 # on the right of [AB]
   if ( K[0] == A[0] ): return abs(K[1]-A[1])
   if ( K[0] == B[0] ): return abs(K[1]-B[1])
   
   AB = [ B[0]-A[0] , B[1]-A[1] ]
   j  = [ 0 , 1 ]
   AK = [ K[0]-A[0] , K[1]-A[1] ]
   dist = VectProd(AK,AB)/VectProd(j,AB)
   if ( dist < 0 ): dist = -dist
   return dist


################################################################################
#
#  Create a random convex hull

np.random.seed(1)

def Rd():
   step = 0.6
   return step * (np.random.random() / 2 + 0.5)

def CreateHull():
   global Upper, Lower,ULeft,URight,LLeft,LRight
   Upper = []
   Lower = []

   theta = 0
   R = 100
   
   while ( theta < math.pi):
      x = R * math.cos(theta)
      y = R * math.sin(theta)
      Upper.append((x,y))
      theta += Rd()
   
   theta = math.pi
   Upper.append((-R,0))
   Upper.reverse()
   
   while ( theta < math.pi * 2):
      x = R *  math.cos(theta)
      y = R *  math.sin(theta)
      Lower.append((x,y))
      theta += Rd()
   
   Lower.append((R,0))
      
   ULeft  = 0
   URight = len(Upper) - 1
   LLeft  = 0
   LRight = len(Lower) - 1

################################################################################
#
#  Draw functions

 
def DrawBorder(border,color):
   X = [i[0] for i in border]
   Y = [i[1] for i in border]
   plt.plot(X,Y, color + '--')
   for j in range(len(X) ):
      plt.plot(X[j], Y[j], color + '.')
      
def DrawPoint(P,color):
   (x,y) = P
   plt.plot(x, y, color + 'o')
   


def sqr(val):
   t = np.sign(val)
   val = np.sqrt(np.abs(val))
   return (t * val) * 20
   
def DrawFunction(LP,mycolor,mid):
  
   n = len(LP)
   for i in range(1,n-1):
      x0 = LP[i-1][0]
      x1 = LP[i][0]
      p = sqr(LSlope(LP,i) ) 
      p2 = sqr(RSlope(LP, i) )
      plt.plot([x0, x1], [p , p ], color= mycolor , linestyle='-' )
      width = 1
      if ( mid == i): width = 5
      plt.plot([x1, x1], [p , p2], color= mycolor , linestyle='-', linewidth= width )
         
def VertLine(Pt):
   (x,y) = Pt
   plt.plot([x, x], [-110 , 110], color= 'r' , linestyle='--' )
   
def drawOld(): 
   plt.cla()  
   for i in range(ULeft,URight+1) : DrawPoint(Upper[i],'r')
   for i in range(LLeft,LRight+1) : DrawPoint(Lower[i],'r')

def drawInfo(Umid,Lmid,CUTx,CUT):
   DrawBorder(Lower, 'k')
   DrawBorder(Upper, 'k')
   
   for i in range(ULeft,URight+1) : DrawPoint(Upper[i],'b')
   for i in range(LLeft,LRight+1) : DrawPoint(Lower[i],'g')
    
   if ( Umid != -1 ) : DrawPoint(Upper[Umid],'r'); print("Upper Mid index: " + str(Umid))
   if ( Lmid != -1 ) : DrawPoint(Lower[Lmid],'r'); print("Lower Mid index: " + str(Lmid))

   DrawFunction(Upper,'r',Umid)
   DrawFunction(Lower,'g',Lmid)
   
   print("---------------------------------------------------")
   if ( CUTx != -1):  VertLine(CUTx)
   if ( CUT != -1):   print(CUT + ' CUT ' )
   
   print ( "Upper indexes: ]" + str(ULeft) + "," + str(URight) + "[")
   print ( "Lower indexes: ]" + str(LLeft) + "," + str(LRight) + "[")
      
      
   plt.text( -50 , 150 , CUT)
   plt.text( -100 , 150 , "ITER " + str(Iter))
      
   
   plt.show()
   plt.pause(0.01)
   time.sleep(sleeptime)
   
################################################################################
#
#  Double Binary Search Algorithm

 
def FindOptimum():
   global ULeft, URight, LLeft, LRight
  
   plt.cla()
   drawInfo(-1,-1,-1,-1)
   
   while ( (URight - ULeft > 1) and ( LRight - LLeft > 1 ) ):
      drawOld()
      # U => decreasing
      Umid = (ULeft+URight) // 2
      Uder = RSlope(Upper,Umid)
      Ux = Upper[Umid][0]
      
      # L => increasing
      Lmid = (LLeft+LRight) // 2
      Lder = RSlope(Lower,Lmid)
      Lx = Lower[Lmid][0]
   
      # CUTTING
      CUT = ""
      if ( Uder-Lder <= 0 ): 
         if  ( Ux <= Lx ): CUT += "L RIGHT  "; CUTx = Lower[Lmid];  LRight = Lmid
         if  ( Lx <= Ux ): CUT += "U RIGHT  "; CUTx = Upper[Umid];  URight = Umid
      if ( Uder-Lder >= 0 ):  
         if  ( Ux <= Lx ): CUT += "U LEFT   ";  CUTx = Upper[Umid]; ULeft  = Umid
         if  ( Lx <= Ux ): CUT += "L LEFT   ";  CUTx = Lower[Lmid]; LLeft  = Lmid
            
      drawInfo(Umid,Lmid,CUTx,CUT)     
   
   print ("###   Double Binary Search TERMINATED")                                                       
   
   if (  LRight - LLeft == 1   ):
      print ("###   Binary search on the upper border")
      Lder = RSlope(Lower,LLeft)
      LxLeft  = Lower[LLeft][0]
      LxRight = Lower[LRight][0]
         
      while (URight - ULeft > 1):
         drawOld()
      
         Umid = (ULeft+URight) // 2
         Uder = RSlope(Upper,Umid)
         Ux = Upper[Umid][0]
         CUTx = Upper[Umid]
         
         if   ( Ux <= LxLeft  ): CUT = "SIMPLE BS - OUTSIDE - UPPER LEFT";   ULeft  = Umid
         elif ( Ux >= LxRight ): CUT = "SIMPLE BS - OUTSIDE - UPPER RIGHT";  URight = Umid
         elif ( Uder-Lder <= 0 ): 
            URight = Umid; CUT = "SIMPLE BS - U RIGHT"
         else: 
            ULeft = Umid;  CUT = "SIMPLE BS - U LEFT"

         drawInfo(Umid,-1,CUTx,CUT)  
         
   else:
      print ("###   Binary search on the LOWER border")
      Uder = RSlope(Upper,ULeft)
      UxLeft  = Upper[ULeft][0]
      UxRight = Upper[URight][0]
         
      while (LRight - LLeft > 1):
         drawOld()
      
         Lmid = (LLeft+LRight) // 2
         Lx = Lower[Lmid][0]
         Lder = RSlope(Lower,Lmid)
         CUTx = Lower[Lmid]
         
         if   ( Lx <= UxLeft  ): CUT = "SIMPLE BS - OUTSIDE - LOWER LEFT";  LLeft = Lmid
         elif ( Lx >= UxRight ): CUT = "SIMPLE BS - OUTSIDE - LOWER RIGHT"; LRight = Lmid
         elif ( Uder-Lder <= 0 ): 
            CUT = "SIMPLE BS - LOWER LEFT"; LRight  = Lmid
         else: 
            CUT = "SIMPLE BS - LOWER RIGHT"; LLeft = Lmid
      
         plt.cla()
         drawInfo(-1,Lmid,CUTx,CUT)  
      
   # remaining four points, brute force approach
   # compute the 4 possibilies and take the max
   V = 0
   Tmax = 0
   t = VertDist(Upper[ULeft],Upper[URight],Lower[LLeft])
   if ( t > Tmax ): V = Lower[LLeft]; Tmax = t
   t = VertDist(Upper[ULeft],Upper[URight],Lower[LRight])
   if ( t > Tmax ): V = Lower[LRight]; Tmax = t 
   t = VertDist(Lower[LLeft],Lower[LRight],Upper[ULeft])
   if ( t > Tmax ): V = Upper[ULeft]; Tmax = t
   t = VertDist(Lower[LLeft],Lower[LRight],Upper[URight])
   if ( t > Tmax ): V = Upper[URight]; Tmax = t
   
   if ( Tmax > 0 ): 
      plt.cla()
      drawInfo(-1,-1,V,"FOUND")
      return Tmax     
   else:  
      print("EEEERRRRROOOOORRRRR")
      return -1
  
################################################################################
#
#  Compute the vertical thickness using a brute force approach
#  This fnt is used to check the result of our algorithm

  
def ComputeLinea():
   umax = len(Upper) - 1
   lmax = len(Lower) - 1
   Tmax = 0
   for u in range(umax-1):
      for l in range(lmax): 
         t = VertDist(Upper[u],Upper[u+1],Lower[l]) 
         Tmax = max(Tmax,t)
         
   for u in range(umax):
      for l in range(lmax-1): 
         t = VertDist(Lower[l],Lower[l+1],Upper[u]) 
         Tmax = max(Tmax,t)
  
   return Tmax
     
################################################################################
#
#  Main Loop

  
go = True
Iter = 0
while(go):
   Iter += 1
   print("###")
   print("######################################################################")
   print("###")
   print("###   Create Random Convex Hull")
   CreateHull()   
   print("###   Compute the maximum vertical thickness") 
   tt = FindOptimum()
   print("###   Checking optimum thickness")
   mm = ComputeLinea()
   print("###   Tv: ",tt)
   print("###   TV ",mm, "Checked value")
   if abs(tt-mm) > 0.001 : 
      go = False
      print("###   VThickness KO")
   else: 
      print("###   VThickness OK")
