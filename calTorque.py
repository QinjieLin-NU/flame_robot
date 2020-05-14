import numpy as np

#read txt file function
def read_txt(filename):
    f = open(filename, "r")
    l = []
    l = [ line.split() for line in f]

    weights = np.array(l)
    return matrix

#n is the index of joint
def networkOutput(weight,offset,scale,n):
    lw = weight[:,n]
    loff = offset[:,n]
    lscale = scale[:,n]

    output = 0
    output += s.hipx.q               - loff[0])  * lscale[0]  * lw[0]



weight = read_txt("standweight.txt")
offset = read_txt("standoffset.txt")
scale = read_txt("standscale.txt")


iweights = weights[:,0]


#get weights, offset, scale