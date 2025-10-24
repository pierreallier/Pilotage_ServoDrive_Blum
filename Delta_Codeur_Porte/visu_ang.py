import matplotlib.pyplot as plt
import math

ang_sortie_reel = [-62.5, 0, 82.5]
red = 2.4
ang_capt_brut = [223, 13, 210]
ang_capt_meuble_brut = [242, 13, 156]

def corr_ang_brut(ang_brut):
    if ang_brut[0] > ang_brut[1]:
        ang_brut[0] -= 360
    if ang_brut[1] > ang_brut[2]:
        ang_brut[1] -= 360
    return ang_brut

def zero_ang(ang_brut):
    ang_zero = ang_brut[1]
    return [ang - ang_zero for ang in ang_brut]

ang_sortie_mes =  [ang/red for ang in zero_ang(corr_ang_brut(ang_capt_brut))]
ang_sortie_meuble_mes =  [ang/red for ang in zero_ang(corr_ang_brut(ang_capt_meuble_brut))]
ang_securite = [-58.3333, 0, 75]

print("Angle sortie corrigé : ",ang_sortie_mes, "plage totale : ", max(ang_sortie_mes)-min(ang_sortie_mes))
print("Angle sortie meuble corrigé : ",ang_sortie_meuble_mes, "plage totale : ", max(ang_sortie_meuble_mes)-min(ang_sortie_meuble_mes))
print("Valeur de correction : ", -corr_ang_brut(ang_capt_brut)[1])
print("Securités : ", [ang*red for ang in ang_securite]," vs +/- 5° : ", [(ang_sortie_mes[0]+5)*red,ang_sortie_mes[1],(ang_sortie_mes[2]-5)*red])

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
for theta in ang_sortie_reel:
    ax.plot([theta*2*math.pi/360, theta*2*math.pi/360], [0,1], 'k')
for theta in ang_sortie_mes:
    ax.plot([theta*2*math.pi/360, theta*2*math.pi/360], [0,1], 'r')
for theta in ang_sortie_meuble_mes:
    ax.plot([theta*2*math.pi/360, theta*2*math.pi/360], [0,1], 'b')
ax.fill([ang_sortie_mes[0]*math.pi/180, ang_sortie_mes[0]*math.pi/180, (ang_securite[0])*math.pi/180, (ang_securite[0])*math.pi/180],[0,1,1,0], color='r', alpha=0.3)
ax.fill([ang_sortie_mes[2]*math.pi/180, ang_sortie_mes[2]*math.pi/180, (ang_securite[2])*math.pi/180, (ang_securite[2])*math.pi/180],[0,1,1,0], color='r', alpha=0.3)
plt.show()