       #include "hmc5883.h"    
020    #include "MK60_i2c.h"    
021    #include  "common.h"    
022         
023    float  x_scale,y_scale,z_scale;//�¶ȱ仯��������    
024    float  HMC5883_lastx,HMC5883_lasty,HMC5883_lastz;    
025    int16  HMC5883_FIFO[3][11]; //�������˲�    
026    int16  HMC5883_maxx=0,HMC5883_maxy=0,HMC5883_maxz=0,HMC5883_minx=-0,HMC5883_miny=-0,HMC5883_minz=-0;//�����Ʊ궨ֵ    
027    uint8  HMC5883_calib=0; //��ʼ����ɱ�־    
028         
029    //ÿ������ϵ����Ӧ�� LSb/Gauss    
030    const int16 counts_per_milligauss[8]=    
031    {    
032      1370,  //+-0.88Ga    
033      1090,  //+-1.3Ga    
034      820,   //+-1.9Ga    
035      660,   //+-2.5Ga    
036      440,   //+-4.0Ga    
037      390,   //+-4.7Ga    
038      330,   //+-5.6Ga    
039      230    //+-8.1Ga    
040    };    
041    /*    
042    int16 HMC5883_GetDoubleData(uint8 Addr)    
043    {    
044      uint16 data=0x0000;    
045      data=i2c_read_reg(HMC5883_I2C_Moudle,HMC5883_ADDRESS, Addr);    
046      data=(uint16)((data<<8)&0xff00);    
047      data+=i2c_read_reg(HMC5883_I2C_Moudle,HMC5883_ADDRESS, Addr+1);    
048      //int16 data;    
049      //data=   ((int16)i2c_read_reg(MPU6050_I2C_Moudle,MPU6050_ADDRESS, Addr)<<8)    
050      //      |(int16)i2c_read_reg(MPU6050_I2C_Moudle,MPU6050_ADDRESS, Addr+1);    
051           
052      return (int16)data;//�ϳ����ݣ�Ϊ�з���������    
053    }    
054         
055    void HMC5883_init(void)    
056    {    
057        i2c_init(I2C0,400*1000); // ��ʼ��I2C0���ڴ��Ĳ�����Ϊ400k    
058        i2c_write_reg(HMC5883_I2C_Moudle,HMC5883_ADDRESS,HMC_CFG1, 0x78);    
059        i2c_write_reg(HMC5883_I2C_Moudle,HMC5883_ADDRESS,HMC_CFG2, 0x00);    
060        i2c_write_reg(HMC5883_I2C_Moudle,HMC5883_ADDRESS,HMC_MOD, 0x00);    
061    }    
062    */    
063         
064    //����ƽ���˲�    
065    void  HMC58X3_newValues(int16 x,int16 y,int16 z)    
066    {    
067      uint8 i,j;    
068      int16 min[3],max[3];    
069      int32 sum_data = 0;    
070      for(j=1;j<10;j++)    
071      {    
072        HMC5883_FIFO[0][j-1]=HMC5883_FIFO[0][j];    
073        HMC5883_FIFO[1][j-1]=HMC5883_FIFO[1][j];    
074        HMC5883_FIFO[2][j-1]=HMC5883_FIFO[2][j];    
075      }    
076      //�Ҽ�ֵ    
077      for(i=0;i<3;i++)    
078      {    
079        min[i] = HMC5883_FIFO[i][0];    
080        max[i] = HMC5883_FIFO[i][0];    
081        for(j=0;j<10;j++)    
082        {    
083          if(max[i]<=HMC5883_FIFO[i][j])    
084            max[i] = HMC5883_FIFO[i][j];    
085          if(min[i]>=HMC5883_FIFO[i][j])    
086            min[i] = HMC5883_FIFO[i][j];    
087        }    
088      }    
089      //ȥ��ֵ��ƽ��    
090      for(i=0;i<3;i++)    
091      {    
092        for(j=0;j<10;j++)    
093        {    
094          sum_data += HMC5883_FIFO[i][j];    
095          HMC5883_FIFO[i][10] = (sum_data - min[i] - max[i])/8;    
096        }    
097        sum_data = 0;    
098      }    
099      if(HMC5883_calib)//У����Ч,�ɼ��궨ֵ    
100      {    
101        if(HMC5883_minx>HMC5883_FIFO[0][10])HMC5883_minx=HMC5883_FIFO[0][10];    
102        if(HMC5883_miny>HMC5883_FIFO[1][10])HMC5883_miny=HMC5883_FIFO[1][10];    
103        if(HMC5883_minz>HMC5883_FIFO[2][10])HMC5883_minz=HMC5883_FIFO[2][10];    
104             
105        if(HMC5883_maxx<HMC5883_FIFO[0][10])HMC5883_maxx=HMC5883_FIFO[0][10];    
106        if(HMC5883_maxy<HMC5883_FIFO[1][10])HMC5883_maxy=HMC5883_FIFO[1][10];    
107        if(HMC5883_maxz<HMC5883_FIFO[2][10])HMC5883_maxz=HMC5883_FIFO[2][10];    
108        //LED_Change();    
109      }    
110    }    
111         
112    //��ȡ�����Ƶ�ǰADCֵ    
113    void HMC58X3_getlastValues(int16 *x,int16 *y,int16 *z)    
114    {    
115      *x = HMC5883_FIFO[0][10];    
116      *y = HMC5883_FIFO[1][10];    
117      *z = HMC5883_FIFO[2][10];    
118    }    
119         
120    //��ȡУ�����ADCֵ    
121    void HMC58X3_mgetValues(float *arry)    
122    {    
123      int16 xr,yr,zr;    
124      HMC58X3_getRaw(&xr, &yr, &zr);    
125      arry[0]= HMC5883_lastx=(float)(xr-((HMC5883_maxx+HMC5883_minx)/2));    
126      arry[1]= HMC5883_lasty=(float)(yr-((HMC5883_maxy+HMC5883_miny)/2));    
127      arry[2]= HMC5883_lastz=(float)(zr-((HMC5883_maxz+HMC5883_minz)/2));    
128    }    
129         
130    //�Ƚϴ�С    
131    int16 min(int16 x,int16 y)    
132    {    
133      if(x<y)    
134        return x;    
135      else    
136        return y;    
137    }    
138    //һ�λ�ȡ16λ���ݣ�2���ֽ�(����һ)    
139    int16 HMC5883L_Axes_data(uint8 reg_addr)    
140    {    
141      int16 xyz_data;    
142      xyz_data = ((int16)i2c_read_reg(HMC5883_I2C_Moudle, HMC5883_ADDRESS, reg_addr)<<8)    
143                  |(int16)i2c_read_reg(HMC5883_I2C_Moudle, HMC5883_ADDRESS, reg_addr+1);    
144      return xyz_data;    
145    }    
146    //��ȡHMC5883L ID    
147    void HMC58X3_getID(int8 id[3])    
148    {    
149           
150      id[0]=i2c_read_reg(HMC5883_I2C_Moudle, HMC5883_ADDRESS, HMC58X3_R_IDA);    
151      id[1]=i2c_read_reg(HMC5883_I2C_Moudle, HMC5883_ADDRESS, HMC58X3_R_IDB);    
152      id[2]=i2c_read_reg(HMC5883_I2C_Moudle, HMC5883_ADDRESS, HMC58X3_R_IDC);    
153    }    
154         
155    /**********************************************************************    
156    ���������     ����ֵ(DOR)    
157    0 -> 0.75Hz  |   1 -> 1.5Hz    
158    2 -> 3Hz     |   3 -> 7.5Hz    
159    4 -> 15Hz    |   5 -> 30Hz    
160    6 -> 75Hz     
161    **********************************************************************/    
162    //����HMC5883L�������������    
163    void HMC58X3_setDOR(uint8 DOR)    
164    {    
165      if (DOR>6)    
166        return;    
167      i2c_write_reg(HMC5883_I2C_Moudle, HMC5883_ADDRESS, HMC58X3_R_CONFA,DOR<<2);    
168    }    
169         
170    //����HMC5883L����    
171    void HMC58X3_setGain(uint8 gain)    
172    {    
173      if (gain > 7)    
174        return;    
175      i2c_write_reg(HMC5883_I2C_Moudle, HMC5883_ADDRESS, HMC58X3_R_CONFB, gain << 5);    
176    }    
177         
178    //����HMC5883L��Ч�Ĺ���ģʽ    
179    void HMC58X3_setMode(uint8 mode)    
180    {    
181      if (mode > 2)//�ж�mode�Ƿ񳬳���Ч��ģʽ��Χ    
182      {    
183        return;    
184      }    
185      i2c_write_reg(HMC5883_I2C_Moudle, HMC5883_ADDRESS, HMC58X3_R_MODE, mode);    
186      DELAY_US(100);    
187    }    
188         
189    //��ȡX��Y��Z������    
190    void HMC58X3_getRaw(int16 *x,int16 *y,int16 *z)    
191    {    
192      uint8 vbuff[6];    
193      vbuff[0] = i2c_read_reg(HMC5883_I2C_Moudle, HMC5883_ADDRESS, HMC58X3_R_XM);    
194      vbuff[1] = i2c_read_reg(HMC5883_I2C_Moudle, HMC5883_ADDRESS, HMC58X3_R_XL);    
195      vbuff[2] = i2c_read_reg(HMC5883_I2C_Moudle, HMC5883_ADDRESS, HMC58X3_R_ZM);    
196      vbuff[3] = i2c_read_reg(HMC5883_I2C_Moudle, HMC5883_ADDRESS, HMC58X3_R_ZL);    
197      vbuff[4] = i2c_read_reg(HMC5883_I2C_Moudle, HMC5883_ADDRESS, HMC58X3_R_YM);    
198      vbuff[5] = i2c_read_reg(HMC5883_I2C_Moudle, HMC5883_ADDRESS, HMC58X3_R_YL);    
199      HMC58X3_newValues(((int16)vbuff[0] << 8) | vbuff[1],((int16)vbuff[4] << 8) | vbuff[5],((int16)vbuff[2] << 8) | vbuff[3]);    
200      *x = HMC5883_FIFO[0][10];
