import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as poly
from shapely.geometry import Polygon,LineString,Point
import random
baslangic_point=(1,1)
bitis_point=(10,10)
bas_acisi=np.array(0,dtype="float64")
genislik=0.5
uzunluk=0.8
tekerlek_yaricapi=0.2
global_orijin_x=np.array(baslangic_point[0])
global_orijin_y=np.array(baslangic_point[1])


x_lim=(0,13)
y_lim=(0,13)
engeller=[[(1,7),(2,7),(2,9),(1,9)],
          [(1,5),(2,5),(2,6),(1,6)],
          [(2,3),(3,3),(3,4),(2,4)],
          [(4,1),(5,1),(4,2),(4,2)],#üçgen
          [(4,5),(5,5),(5,6),(4,6)],
          [(4,8),(5,8),(5,9),(4,9)],
          [(3,11),(4,11),(4,12),(3,12)],
          [(6,1),(7,1),(7,3),(6,3)],
          [(6,5),(8,5),(8,6),(6,6)],
          [(6,9),(7,9),(6,11),(6,11)],#üçgen
          [(9,7),(10,7),(9,8),(9,8)]]#üçgen
engeller=np.array(engeller,dtype=float)
class Harita():
    def __init__(self,start_point,finish_point,x_lim,y_lim,engeller):
        self.start_point=start_point
        self.finish_point=finish_point
        self.x_lim=x_lim
        self.y_lim=y_lim
        self.engeller=engeller
        self.poly_engelller=[Polygon(engel) for engel in self.engeller]
    def cevre(self):
        fig,((self.eks1,self.eks2),(self.eks3,self.eks4))=plt.subplots(2,2,figsize=(16,16))
        self.eks1.set_xlim(*self.x_lim)
        self.eks1.set_ylim(*self.y_lim)
        self.eks1.scatter(*self.start_point,color="green",label="baslangic",s=100,zorder=5,alpha=0.9)
        self.eks1.scatter(*self.finish_point,color="red",label="bitis",s=100,zorder=5,alpha=0.9)
        self.eks1.set_aspect("equal")
        self.eks1.set_title("2B Harita")
        self.eks1.set_xlabel("x (metre)")
        self.eks1.set_ylabel("y (metre)")
        self.eks1.grid(True,linestyle="--",color="blue",alpha=0.5,zorder=1)
        self.eks1.legend(loc="best")
        
    def engel_ekle(self):
        for a in self.engeller:
         kapali_engeller=poly(a,closed=True,facecolor="gray",edgecolor="black",zorder=5,alpha=1)
         self.eks1.add_patch(kapali_engeller)
        
class dif_suruslu_robot():
    def __init__(self,genislik,uzunluk,tekerlek_yaricapi,bas_acisi,global_orijin_x,global_orijin_y,dt):
       #burada diferansiyel roboutun gerekli parametrelerini tanımladım.
       self.genislik=genislik
       self.uzunluk=uzunluk
       self.tekerlek_yaricapi=tekerlek_yaricapi
       self.bas_acisi=bas_acisi
       self.global_orijin_x=global_orijin_x
       self.global_orijin_y=global_orijin_y
       self.dt=dt
     
       self.rota_x=[self.global_orijin_x]#burada globaldeki rotasını bir listede tuttum.
       self.rota_y=[self.global_orijin_y]#burada globaldeki rotasını bir listede tuttum.
    def hareket_ettir(self,x_local_hiz,omega):
       
       self.x_local_hiz=x_local_hiz #holonomik olmayan olduğu için sadece hızın x bileşeni var
       self.omega=omega
       self.global_orijin_x=self.global_orijin_x+(self.x_local_hiz)*(np.cos(self.bas_acisi))*self.dt #burada ayrık bir integral işlemi uyguladım.
       self.global_orijin_y=self.global_orijin_y+(self.x_local_hiz)*(np.sin(self.bas_acisi))*self.dt 
       self.bas_acisi=self.omega*self.dt+ self.bas_acisi
       self.bas_acisi=np.arctan2(np.sin(self.bas_acisi),np.cos(self.bas_acisi))
       self.rota_x.append(self.global_orijin_x)
       self.rota_y.append(self.global_orijin_y)
    def robotu_yerlestir(self,eks1):
         self.rotasyon=np.array([[np.cos(self.bas_acisi),-np.sin(self.bas_acisi),self.global_orijin_x],[np.sin(self.bas_acisi),np.cos(self.bas_acisi),self.global_orijin_y],
         [0,0,1]] )
         self.kose1=self.rotasyon.dot(np.array([self.uzunluk/2,self.genislik/2,1]))[0:2]
         self.kose2=self.rotasyon.dot(np.array([self.uzunluk/2,-self.genislik/2,1]))[0:2]
         self.kose3=self.rotasyon.dot(np.array([-self.uzunluk/2,-self.genislik/2,1]))[0:2]
         self.kose4=self.rotasyon.dot(np.array([-self.uzunluk/2,self.genislik/2,1]))[0:2]
         self.koseler=np.array([self.kose4,self.kose3,self.kose2,self.kose1])
         robot_kapali=poly(self.koseler,closed=True,facecolor="orange",edgecolor="black",zorder=4,alpha=1) #robotu öteleme ve rotasyona ayarlanacak şekilde çizdim. 
         eks1.add_patch(robot_kapali)
         eks1.arrow(self.global_orijin_x,self.global_orijin_y,self.uzunluk*np.cos(self.bas_acisi),self.uzunluk*np.sin(self.bas_acisi),head_width=0.1,head_length=0.1,fc="yellow",ec="black",zorder=5) #robotun önünü göstermek için bir ok çizdim.
    def lidar(self,poly_engeller,kapsam,isin_sayisi):
       self.poly_engeller=poly_engeller
       self.kapsam=kapsam
       self.isin_sayisi=isin_sayisi
       self.lidar_noktalari=[]
       self.robotun_merkezi=Point(self.global_orijin_x,self.global_orijin_y)
       for i in range(self.isin_sayisi):
          
          
          aci=(i*(2*np.pi))/self.isin_sayisi
          lidar_acisi=self.bas_acisi+aci
          lidar_bitis_x=self.global_orijin_x+self.kapsam*np.cos(lidar_acisi)
          lidar_bitis_y=self.global_orijin_y+self.kapsam*np.sin(lidar_acisi)
          isin=LineString([(self.global_orijin_x,self.global_orijin_y),(lidar_bitis_x,lidar_bitis_y)])
          en_yakin_nokta=None
          min_mesafe=self.kapsam
          for engel in self.poly_engeller:
            if isin.intersects(engel):
                kesim_noktasi=isin.intersection(engel.boundary)
                if kesim_noktasi.is_empty:
                   continue
                if kesim_noktasi.geom_type=="MultiPoint":
                   kesim_noktasi=list(kesim_noktasi.geoms)
                elif kesim_noktasi.geom_type=="Point":
                     kesim_noktasi=[kesim_noktasi]
                else:
                     continue
                
                for nokta in kesim_noktasi:
                   if self.robotun_merkezi.distance(nokta)<min_mesafe:
                      min_mesafe=self.robotun_merkezi.distance(nokta)
                      en_yakin_nokta=(nokta.x,nokta.y)
                
          if en_yakin_nokta is not None:
               self.lidar_noktalari.append(en_yakin_nokta)
       return (self.lidar_noktalari)           
    def apf_kontrol(self,hedef_x,hedef_y,lidar_noktalari,cekme,itme,etki_alani,max_hiz,max_acisal_hiz):
         self.max_hiz=max_hiz
         self.max_acisal_hiz=max_acisal_hiz
         #robotumuz dikdörgen şeklinde. Bu yüzden gövdeni çarpması engelemek vasıtasıyla robotun merkezinden bir çember tanımladım. 
         #bu ölçü dikdörgeni içine alacak en küçük yarıçaplı çember olmalı.
         robot_cember_yaricapi=np.hypot(self.uzunluk/2,self.genislik/2)
         x_de_oteleme=hedef_x-self.global_orijin_x
         y_de_oteleme=hedef_y-self.global_orijin_y
         x_de_cekim=x_de_oteleme*cekme
         y_de_cekim=y_de_oteleme*cekme
         x_de_itme=0 # lidarda bir şey gözükmezse itme kuvveti sıfır olur
         y_de_itme=0 # lidarda bir şey gözükmezse itme kuvveti sıfır olur
         if lidar_noktalari is not None:
            for nokta in lidar_noktalari:
               x_de_oteleme_engele_merkez_arasi=self.global_orijin_x-nokta[0]
               y_de_oteleme_engele_merkez_arasi=self.global_orijin_y-nokta[1]
               uzaklik_engele_ile_robot_arasi=np.hypot(x_de_oteleme_engele_merkez_arasi,y_de_oteleme_engele_merkez_arasi)-robot_cember_yaricapi
               mesafe=np.hypot(x_de_oteleme_engele_merkez_arasi,y_de_oteleme_engele_merkez_arasi) # vekrörel kuvveti bulmak için lazım.
               if uzaklik_engele_ile_robot_arasi==0:
                  uzaklik_engele_ile_robot_arasi=0.001 #sıfıra bölünemeyeceği için.
               if uzaklik_engele_ile_robot_arasi<=etki_alani:
                  kuvvet=itme*(1/uzaklik_engele_ile_robot_arasi-1/etki_alani)*(1/uzaklik_engele_ile_robot_arasi**2)
                  x_de_itme+=kuvvet*(x_de_oteleme_engele_merkez_arasi/mesafe)
                  y_de_itme+=kuvvet*(y_de_oteleme_engele_merkez_arasi/mesafe)
         x_de_kuvvet_toplam=x_de_cekim+x_de_itme   
         y_de_kuvvet_toplam=y_de_cekim+y_de_itme
         aci_hata=np.arctan2(y_de_kuvvet_toplam,x_de_kuvvet_toplam)-self.bas_acisi
         aci_hata=np.arctan2(np.sin(aci_hata),np.cos(aci_hata)) #-180 ile 180 arasında olması için.
         omega=aci_hata*1.5 # 1.5 yerine dt miktarıda yazabilirdim öyle olsaydı 1 saniyede tam olarak hedefe yönelirdi. Ben biraz daha hızlı yönelmesi için 1.5 yazdım.
         v=self.max_hiz*np.cos(aci_hata) # hedefe yöneldiyse hedef tam gaz kilitlenir.
         v=np.clip(v,0,self.max_hiz) # robotun engelden kaçıp da geriye dönmesini engeller. ve holonmomik olmadığı için sadece ileri gider.
         omega=np.clip(omega,-self.max_acisal_hiz,self.max_acisal_hiz)
         return (v,omega)

class dugum():
      def __init__(self,x,y,uye=None):
         self.x=x
         self.y=y
         self.uye=uye
class RRT():
   def __init__(self,baslangic_noktasi,bitis_noktasi,x_lim,y_lim,shapely_engeller,adim_miktari=1.2,max_iterasyon=4000):
       self.baslangic_noktasi=dugum(baslangic_noktasi[0],baslangic_noktasi[1])
       self.bitis_noktasi=dugum(bitis_noktasi[0],bitis_noktasi[1])
       self.x_lim=x_lim
       self.y_lim=y_lim
       self.shapely_engeller=shapely_engeller
       self.adim_miktari=adim_miktari
       self.max_iterasyon=max_iterasyon
       self.dallar=[self.baslangic_noktasi]
   def rastgele_nokta(self):
       if random.random()>0.2:
           return dugum(random.uniform(self.x_lim[0],self.x_lim[1]),random.uniform(self.y_lim[0],self.y_lim[1]))
       return dugum(self.bitis_noktasi.x,self.bitis_noktasi.y) 
   def en_yakin_dugum(self,rastgele_nokta):
       mesafeler=[np.hypot(dal.x-rastgele_nokta.x,dal.y-rastgele_nokta.y) for dal in self.dallar] 
       return self.dallar[mesafeler.index(min(mesafeler))]
   def carpisma_kontrol(self,dugum1,dugum2):
         isin=LineString([(dugum1.x,dugum1.y),(dugum2.x,dugum2.y)])
         for engel in self.shapely_engeller:
               if isin.intersects(engel) or engel.contains(Point(dugum2.x,dugum2.y)): 
                  return True
         return False
   def planla(self): 
       print("RRT algoritmasi calisiyor") 
       for _ in range(self.max_iterasyon):
           rastgele_nokta1=self.rastgele_nokta()
           en_yakindugum=self.en_yakin_dugum(rastgele_nokta1)
           aci=np.arctan2(rastgele_nokta1.y-en_yakindugum.y,rastgele_nokta1   .x-en_yakindugum.x)
           yeni_dugum=dugum(en_yakindugum.x+self.adim_miktari*np.cos(aci),en_yakindugum.y+self.adim_miktari*np.sin(aci),en_yakindugum)
           rota=[]
           if not self.carpisma_kontrol(en_yakindugum,yeni_dugum):
               self.dallar.append(yeni_dugum)
               yeni_dugum.uye=en_yakindugum
           if np.hypot(yeni_dugum.x-self.bitis_noktasi.x,yeni_dugum.y-self.bitis_noktasi.y)<=self.adim_miktari:
               self.bitis_noktasi.uye=yeni_dugum
               referans_noktasi=self.bitis_noktasi  
               while referans_noktasi is not None:
                     rota.append((referans_noktasi.x,referans_noktasi.y))
                     referans_noktasi=referans_noktasi.uye
               return rota[::-1]
       return None           
       
 

            
       





if __name__=="__main__": 
 baslangic_point=(1,1)
 bitis_point=(10,10)
 degisme_ihtimali_olmayan_bitis_point=(10,10)
 bas_acisi=np.array(0,dtype="float64")
 genislik=0.25
 uzunluk=0.4
 tekerlek_yaricapi=0.2
 global_orijin_x=np.array(baslangic_point[0])
 global_orijin_y=np.array(baslangic_point[1])


 x_lim=(0,13)
 y_lim=(0,13)
 engeller=[[(1,7),(2,7),(2,9),(1,9)],
          [(1,5),(2,5),(2,6),(1,6)],
          [(2,3),(3,3),(3,4),(2,4)],
          [(4,1),(5,1),(4,2),(4,2)],#üçgen
          [(4,5),(5,5),(5,6),(4,6)],
          [(4,8),(5,8),(5,9),(4,9)],
          [(3,11),(4,11),(4,12),(3,12)],
          [(6,1),(7,1),(7,3),(6,3)],
          [(6,5),(8,5),(8,6),(6,6)],
          [(6,9),(7,9),(6,11),(6,11)],#üçgen
          [(9,7),(10,7),(9,8),(9,8)]]#üçgen
 engeller=np.array(engeller,dtype=float)
 harita1=Harita(baslangic_point,bitis_point,x_lim,y_lim,engeller)
 harita1.cevre()
 harita1.engel_ekle()
 robot1=dif_suruslu_robot(genislik,uzunluk,tekerlek_yaricapi,bas_acisi,global_orijin_x,global_orijin_y,0.1)
 plt.ion()   #canli izlemek için interaktif modu actim.
 RRT1=RRT(baslangic_point,bitis_point,x_lim,y_lim,harita1.poly_engelller)
 rota=RRT1.planla()
 a=0
     
 
 while True:
      
     if np.hypot(robot1.global_orijin_x-degisme_ihtimali_olmayan_bitis_point[0],robot1.global_orijin_y-degisme_ihtimali_olmayan_bitis_point[1])<0.1:
         print("Hedefe ulasildi")
         break
      
     if rota is None:
        print("Rota bulunamadi")
        bitis_point=(10,10)
     else:
        bitis_point=rota[a] #rota listesinin ilk elemanı sıradaki hedef nokta olur.
        if np.hypot(robot1.global_orijin_x-bitis_point[0],robot1.global_orijin_y-bitis_point[1])<0.1: #hedefe yaklaşıldığında sıradaki hedefe geçilir.
            a+=1

     harita1.eks1.cla() 
      
     (v,omega)=robot1.apf_kontrol(bitis_point[0],bitis_point[1],robot1.lidar(harita1.poly_engelller,5,36),cekme=50,itme=0.3,etki_alani=0.09,max_hiz=0.25,max_acisal_hiz=np.pi/1.3)
     robot1.hareket_ettir(v,omega)
     
     lidar_sonuclari=robot1.lidar(harita1.poly_engelller,5,36)
     if lidar_sonuclari is not None:
        for lidar_sonucu in lidar_sonuclari:
           harita1.eks1.scatter(*lidar_sonucu,color="purple",s=20,zorder=6,alpha=0.7)
           (x,y)=lidar_sonucu
           harita1.eks1.plot([robot1.global_orijin_x,x],[robot1.global_orijin_y,y],color="red",linestyle="--",zorder=5,alpha=0.7)
     if rota is not None:
         harita1.eks1.plot(*zip(*rota),color="cyan",label="RRT Rotasi",zorder=6,alpha=0.7)
         
     harita1.eks1.plot(robot1.rota_x,robot1.rota_y,color="blue",label="rota",zorder=7,alpha=0.7)
     harita1.eks1.set_xlim(*harita1.x_lim)
     harita1.eks1.set_ylim(*harita1.y_lim)
     harita1.eks1.scatter(*harita1.start_point,color="green",label="baslangic",s=100,zorder=5,alpha=0.9)
     harita1.eks1.scatter(*harita1.finish_point,color="red",label="bitis",s=100,zorder=5,alpha=0.9)
     harita1.eks1.set_aspect("equal")
     harita1.eks1.set_title("2B Harita ve APF Kontrolü")
     harita1.eks1.set_xlabel("x (metre)")
     harita1.eks1.set_ylabel("y (metre)")
     harita1.eks1.grid(True,linestyle="--",color="blue",alpha=0.5,zorder=1)
     harita1.engel_ekle()
     robot1.robotu_yerlestir(harita1.eks1)
     
     plt.draw()
     harita1.eks1.legend(loc="best")
     plt.pause(0.01)


 plt.ioff() #interaktif modu kapattim.
 plt.show()



         


        










