

141 0411


1 ���ڳ���beep��Ե�������������⣬��ʱδ�ҵ����ԭ������ʾ������ʾbeep��Ϣ�����ڴ�ӡbeep��switch���ݡ��͵�����ʱ��������

2 uart4 ��Ϊ���ڡ���option�е� c/c++ѡ��������ã�,STM32F103VCT6_MCU����ѡ���Ƿ�ΪVCϵ�С�ȥ�������VBT6.





141 0327

�޸�оƬΪstm32f103vbt6
��֮ǰ 256k��Ϊ�� 128K

�����ص�ַ��Ҫ���и��ģ�

0- 0x7fff ռ��1/4

0x7fff  - 0xfffe  1/4

ʣ�ಿ�ֵ� 1ffff ʣ��1��2


bootloader��app֮��Ķ�д�����ַ��Ҫ����

�����ң����ip��Ҫ�洢��ַ�޸�

��ת��ַ��Ҫ���ģ�0x8000��Ҳ����0x8008000����СΪ0x18000��

stm32f103vbt6 ��ʱ��ֻ��4·������ֻ����·��

tim5��Ϊtim4
uart4��Ϊuart1
ȥ�����ڵĹ��ܣ��Ժ�ֻ��debug�ˡ�



141 0308

ȷ�ϵ�ѹ������⣬��Ҫ�Ƕ����ܵ�ͨ��ѹ�Լ�mos��ͨ�����ĵ�ѹ���١�ʵ���ϵĲ���ֵ���ǱȽ϶�Ӧ�ġ�
�޸ĳ�ʼ��Ĭ����ֵΪ168��16.8V��

����¶ȼ�⴫�����������¶��Ƿ������������Ϊ������ʵ�ʲ���Ϊǻ���¶ȡ�


141 0305

����gps�쳣����Ȼ���Կ��ơ�

��gps�쳣�󣬹ر�һ��led��moto�����������ܹ��������ơ�led toggle��


�ڹ켣�����У�����ֵεεε���������Ҫ�ҵ�������д�����ʱδ�ҵ�ԭ��Ҳδ�ܸ��֡�

��������״̬����Ҫ�����������յ��������� ң�����Ѿ������޸ġ�
	��Ӧң�������յ����������󣬲�Ӧ�÷���ֹͣ���


//

140�汾

-1 �����д���360��ʱ��ԭ����ת�����⡣ 
	�������Կ��ǣ��������ڱȽϷ���Щ��ȡ��90����ת����������ת�������г��ԡ�
	�Ż��жϣ�360�ȸ��������ַ�ʽ��
	
-2 �����ڵ��������У�����rf��ʧ�źţ�Ӧ��ֹͣ�󣬷�������㣬�����������趨�������·����ok
-3 ������ת�Ƕ�Ϊ90�ȡ� // ����ȥ�����ܡ��Ѿ�ȥ����ok
4 ����������ת����ĽǶ����ݡ������ҽǶȲ�ֵ�����ʱ��������ת�Ƕ�С�ķ���//��ͬ��1 1�ĸ�����ϣ�4��ӦҲ���ĺ��ˡ���ԭ���Ѿ��ĺõĳ���
	�������������Ƚ��ڲ�������ٿ��Ǹ���Ϊ����ԭ����ת��

5 ���ӵ�������������⡣//�Ѿ����� ��ʱ�����ڵ�ѹλ�ã�Ϊ��������ֵ*1000.

6 compassģ��ż����������Ϊ0. //������

7 gps�޷�׼ȷ�͸ߵµ�ͼ����һ�¡��ٴν��в��ԡ�

8 �����p�����£���Ӧ�ٶ���΢���ڶٴ�С���Ҫ���п��Ǹ�����ز����������Ǹ��ĽǶȷ�Χ��Ӧ�ñ�5��СһЩ��
	��ʱ�����޸ġ�




������- 3��1��

��Ҫ����һ����صĲ���ϵͳ����freertos��



139�汾 2017 0222

����������ش���
����gps���⣬��������ddmm.mmmm����Ϊdddd.dddd��ʽ��

138�汾 2017 0219

1 �������ӱ�������gpsģ��Ĺ���
	�Ѿ��޸Ľ���������ͨ����
˵����
	ֻ���ԭʼ��ģ�飬���ϼ����á�ԭʼģ��δ����������ã�������Ӧ��Ϊ9600��������6M��M7���Կ��ã�δ������ԡ�m8�ݲ����á���װֻ����6m��
2 ����ǰ�����л���ǰ�������޷��ر�ת��Ƶ����⡣	

137�汾 2017 0218

1 ң�ط�����ʱ�����ǽ����ֶ�����ģʽ����Ҫ�����˳��Զ�����ģʽ���ȴ�ң�ؽ�����Ȼ���ٽ����Զ�ģ�顣
2 ������ʱ������Ƕ���ֵ����30�ȣ���ԭ��ת�䡣���ת��뾶��������⡣
3 ǰ�����̾�����ֱ�ߣ�ͨ���ṹ�޸ģ���ʱ����������ġ����Ǻ��ڸ�����Ҫ�������compassʵʱУ׼����΢�鷳��
4 oled��ʾ���ݿͻ�������и��ġ�// #define DEBUG_GPS �л�����ģʽ�Ϳͻ�����ģʽ
5 �͵����������ģ�����Ϊһ����
6 �ͻ�����ң�������򿪹�ת�����鲻�ѣ�Ҫ��ҡ�˸�Ӧ��Χ���������в��ԣ��ٿ��ǡ�
		�Ѿ���������޸ģ���Ҫʵ�ʲ��ԡ�20170220�Ѿ�����ͨ����
7 ÿ��beep���ͣ�Ҫ���һ�����ڣ����ܳ��ȼ�Ͻ��档



137�汾 2017 0216

�Ľ��µ�pcb
Ӧ�ͻ���������β�Ƶ���˸��


136�汾 2017 0116

�޸�ԭ��ת�䣬ָʾ�Ʋ��������⡣
����δ�����εĿ��ȹ���





136�汾 2017 0112

1 ���߶Ͽ���ʱ�򣬷������ٴν��붪ʧgps��������εεε��졣����Ϊcontrol��beep����ΪsetBeepBlood = 2����Ҫȥ�����λ�ã��ѽ����
2 ���ݿͻ����󣬸�������״̬Ϊԭ�ش�ת�������ɲ�ͬ���ٶ��ļ���

			if((AS14B_receive_buf[6]  >= BAR_CENTER_POSITION_MIN )&&(AS14B_receive_buf[6]  <= BAR_CENTER_POSITION_MAX ))  // 3:00
							Moto_Direction_Turn( MOTO_RIGHT,  100, 0); 
			if((AS14B_receive_buf[6]  >= BAR_CENTER_POSITION_MIN )&&(AS14B_receive_buf[6]  <= BAR_CENTER_POSITION_MAX ))  // 3:00
							Moto_Direction_Turn( MOTO_RIGHT,  0, 100);



3 ���ݿͻ�����������led�ƣ����˶��е������������ȷ��ʵ�ʻ�����Ҫ��������Ҳ�Ƶ���˸��ʵ���ϲ��Ϊ��һ��ģ��ݲ��޸ġ�


135�汾 2017 0103

1 ����������ʱ�����������ʱ�򣬽�beep��������Ϊ0.
2 ���������У��Ƕ�ת����30�����ڣ����ٽ���led����˸��
3 �޸��˿���distant�����쳣�����⡣
4 ����һ����־����ʾ������������ң�ضν���Э��������ܶ�Ӧ����������ֹͣ�����⡣
5 adc��ⲻ��Ӧ�������쳣���Ѿ��޸Ķ�Ӧ����Ϊ200k������ܸ���������
6 ���й����У�����compass��gps���жϡ����һЩ�쳣״̬�����⡣
7  ====>  ���պ���������ֽ�����󣬻��߽��մ��󡣵����޷����Ƶ�����? ===��?




134�汾 1231

1 �޸�moto�����ٵ�0���жϣ�����Ҫ+����100�ġ��жϴ����Ѿ�������
2 �ݽ��ٶȣ���ѡ���޸Ķ�Ӧ�궨�塣



134�汾 1230


1. �Զ�У׼���޸�bug��else if(cmd == 0).
2. �������״̬�£��ı���״̬����£���ʱ���ɼ�adc��ѹֵ��100ms��
3. ˲��ǰ�����ҵ��˶����ᵼ�����ݶ�ʧ����Ҫ�����
			 if((AS14B_receive_buf[6]  <= BAR_CENTER_POSITION_MIN)&&(AS14B_receive_buf[6]  >= 0) )
4. ����gps�������쳣ԭ��
		a �Ӵ���rx tx
		b ���������ô���
		c gps���ö�Ӧģʽ����gps��ʽ������gps��������������
		d ģ����
		e �ϵ��쳣
		f ����
5 ���ת������ƽ����ʽ���ԣ��������ط��ȡ�5ms�ı�10��pwmλ������������Ҫ50ms����ߵ�������߳�����Ҫ100ms��

6 �޸��°汾Э�顣������ 1230��


133�汾 1226

1 �ع�����
2 �޸��˵εεΣ��͵εν�������⡣
3 �޸��������޷�׼ȷ�ж�compass״̬�����⡣ >=0x30
4 ������������λ���⣬��ʱδ�鿴������ʱ���ã��ȴ��ٴη�����
5 �޸����Э����롣
6 ����gps���㣬
7 ������Э��04�������������ݣ���ʱ�޸Ļ�ȥ��С�����¼���Ҳ��Ҫȥ��ܡ�
8 ����heatbeats++�������ӵ�3������������Ķ��ĺá���ʱ���ټ�⡣
9 GPS���ݷ��͹�ȥ�������ʧ�ܣ����Խ���app�����жϺʹ����ˡ�
10 ���ӶԶ��ǣ���gps����rf��������Ĵ���check_star_and_beep�н������ѡ�����Ҫ����

��㼰���������λ�ã���Ҫ�����жϺͲ��ԡ�


�ع���أ�
 ����ϲ������á���UART�ࡣ
 �����ϲ��������������Ƶĺ����������Ĵ��η�ʽ��
 �ṹ���������������ҪΪ������Ϣ����������Ϊ���������c�ļ�����
 ��������޸ĵĲ��֣����ú궨��ķ�ʽ�������޸ĺ���ֲ��
 �������������շ���������
 ��ʹ��ö�ٵĵط������������ú궨�塣
 beep�ķ�װ�����ԸĶ�Ϊ���ȼ���ʽ�������Ⱥ����ʽ��
 ȥ����ز�����Ҫ�Ĵ��롣5883 l298n cc1101





1206  132�汾

1 ���ݽӿ�����������	LEDC_ALL(1);	LEDC_ALL(0);	ʵ��ң�ؿ���ledȫ����ȫ��
2 ��oled����ע�ʹ򿪡��������޸Ĵ��룬ʹˢ�¾��ֵ�ÿ�������С�������Ҫ���ԣ��Ƿ񳬹����Ź�ʱ�䡣oled����������ʾ�������Զ�����룬��ɡ�
3 ���Ӱ汾��ʾ��Ŀǰ�� 132��Ϊ�汾�š�
4 led���ݴ����˶�������صĿ��ƣ�����Ӧ���Ѿ����á��������⣬�����ʵ����ġ�
5 ����gps���ݵĴ����Է�װ����Ҫ���޸ġ�Post_GPS_Data(); // ���汾��ʱ������һ�����ơ�
6 beep�����ĸ��ģ��Ѳ��ġ��ʵ����ģ���ȷ�����Դ���ִ��Ϊ׼��


1106  131


���˺ܶ����
��Ҫ���޸����ߣ������Э�顣

����Ҫ�޸ģ��͹��Ļ��Ѻ�����������

�¸��汾��Ҫ������

���������ط����ơ�
ԭ�켣�����ĳ����װ��


1016  124 

�����°汾�����޸�

1 ���ӻ�ȡң�ص�ַ
	AS14B_Get_ADDR();  // 1sʱ�����ڻ�ȡ��ַ
	
	

2 ���ӷ������յ������������Ӧ��

void Respond_Back(void)
�����������Ҫ���

3 ������ص�ָ��ĺ��壬���廹��Ҫϸ����ȷ��



#define FEED_BIN_D  6
#define FEED_BIN_C	5
#define FEED_BIN_B  4
#define FEED_BIN_A  3
#define FISH_HOOK 2 
#define JOY_STICK 1 //���⣿ 

#define START_GPS 7
#define CH1 8
#define CH2 9
#define CH3 10
#define CH4 11
#define GO_SET 12  //   ����
#define GAMPASS 13
#define HEAD_TAIL_LIGHT 14
#define WITH_LIGHT 15
#define ALL_LIGHT 16


// ���� ��ȷ��
#define AUTO_GO 17
//#define GO_SET 18
#define GO_BACK_BY_TAIL 18 //Բ�켣����

#define GO_BACK 21

4 ���ص��������õ��������Ҫ����ϸ���͸��ġ�


0906 122
�ٴ���������
ɾ��������Ҫ���ĵ���������ص�c,h�ļ���





0905
�����ʽ

0904
121 
�޸������¶���ֵ������⡣


0903 
 120 ���ӱ��ص�ѹջ�����������޸��Զ� ����linkstack.c


 
  ����GPSʵʱλ�÷��ʹ��롣 10s����һ�Ρ�

  ���Ӽ��ܵĳ�����secret.c
  
  
  
  
  
void Go_Back_By_Trail(void) // ע�ⵥλ��ת��
{
	double distant_by_trail;
	static double tail_dimensionality = 0, tail_longitude = 0;
	
	
		if(flag_go_back_by_trail == 1) //ң�����յ�����֮�󣬸ı��־λ
		{
	//	 (new_dimensionality, new_longitude, old_dimensionality, old_longitude);
		
			if(distant_test < 3)
			{	
				pop_linkstack(lsp,&old_dimensionality,&old_longitude);
				if((old_dimensionality == -1) && (old_longitude == -1))
				{		
						flag_go_back_by_trail = 0;
						tail_dimensionality = sure_dimensionality;
						tail_longitude = sure_longitude;
				}
			}
			
		}
		else
		{
			if(RefreshCnt>50) // �ж��Ƿ�200ms������һ���ж��Ƿ񵽴�������
			{
				distant_by_trail  = get_distance(tail_dimensionality,tail_longitude ,sure_dimensionality,sure_longitude);
				if(distant_by_trail > 3)
				{
					if((sure_dimensionality != 0) && (sure_longitude != 0))
				    push_linkstack(lsp,&sure_dimensionality,&sure_longitude);
						tail_dimensionality = sure_dimensionality;
						tail_longitude = sure_longitude;
				}
			}	
		}			
}


  
  
     typedef  union result
     {
         double d;
         unsigned char data[8];
     }union_GPS_data;
 
   union_GPS_data r1,r2;


     r1.d = 234.56778;
	  printf("byte0= %d byte1= %d byte2= %d byte3= %d r1=%f r2=%f \n",r1.data[0],r1.data[1],r1.data[2],r1.data[3],r1.d,r2.d);
     r2.data[0]=r1.data[0];
     r2.data[1]=r1.data[1];
     r2.data[2]=r1.data[2];
     r2.data[3]=r1.data[3];
	 r2.data[4]=r1.data[4];
     r2.data[5]=r1.data[5];
     r2.data[6]=r1.data[6];
     r2.data[7]=r1.data[7];
    printf("byte0= %d byte1= %d byte2= %d byte3= %d r1=%f r2=%f \n",r1.data[0],r1.data[1],r1.data[2],r1.data[3],r1.d,r2.d);
  
  		GPS_dimensionality.d =  sure_dimensionality ;
				GPS_longituder.d  = sure_longitude ;
				
				USART_SendData(USART3, 0x00); // addr
				USART_SendData(USART3, 0x03); //type
				
				for(i = 0; i < 8; i ++)
				{
				USART_SendData(USART3, GPS_dimensionality.data[i]);
				}
					for(i = 0; i < 8; i ++)
				{
				USART_SendData(USART3, GPS_longituder.data[i]);
				}
			
				
					USART_SendData(USART3, 0x0a);
  
 
0809
 119 �汾


0809
 118 �汾



��ʾ�¶�

����led��֮ǰ��led���������ϵĸ��š���ʱ����.

#define LL_LED1_OFF		//	digitalHi(GPIOB,GPIO_Pin_1)
#define LL_LED1_ON		//	digitalLo(GPIOB,GPIO_Pin_1)


�����������������ˣ�������һ�ᡣ


0808
 117 �汾
 
 �������͵Ĵ����޸ġ�
 ������ʾ�������ܹ�����������
 
 
 ���ձ��˴�������д�Ĵ��룺
 
#define BAR_CENTER_POSITION_MAX 55  //0x32   max 102  min 0
#define BAR_CENTER_POSITION_MIN 45  //0x34   max 102 min 0

					if(	(AS14B_receive_buf[6]  <= BAR_CENTER_POSITION_MAX) &&
						(AS14B_receive_buf[6]  >= BAR_CENTER_POSITION_MIN) &&
						(AS14B_receive_buf[7]  <= BAR_CENTER_POSITION_MAX) &&
						(AS14B_receive_buf[7]  >= BAR_CENTER_POSITION_MIN))//��������
					{
						Moto_Direction_Turn( MOTO_STOP, 0, 0);
					}
					else if( AS14B_receive_buf[6]  >= BAR_CENTER_POSITION_MAX)
					{
						Moto_Direction_Turn( MOTO_GO, (AS14B_receive_buf[6] -50)*2, (AS14B_receive_buf[6]-50)*2);
					}
					else if( AS14B_receive_buf[7]  >= BAR_CENTER_POSITION_MAX)
					{
						Moto_Direction_Turn( MOTO_ROLL_LIGHT,  (AS14B_receive_buf[7]-50)*2,(AS14B_receive_buf[7]-50)*2);
					}
					else if( AS14B_receive_buf[6] <= BAR_CENTER_POSITION_MIN)
					{
						Moto_Direction_Turn( MOTO_BACK, (50 - AS14B_receive_buf[6])*2, (50 - AS14B_receive_buf[6])*2);
					}
					else if( AS14B_receive_buf[7] <= BAR_CENTER_POSITION_MIN)
					{
							Moto_Direction_Turn(MOTO_ROLL_RIGHT,  (50 - AS14B_receive_buf[7])*2, (50 - AS14B_receive_buf[7]) * 2);
					}


0808
 116 �汾
 ������ʧ֮����Ҫ�Զ��ܹ�������
 ���������Ƽ��ϲ֣��㹳��ָ����ơ�


0807
115�汾

����δ������㣬���Կ�����һ��Ѱ����λ����Ϊһ����ѹ��ֵ����Ҫ�ٲ��ԡ�

0807 

114�汾



����δ���������յ㣬���޷����յ㼰��㷽��ǰ����

���δ�����������յ㣬�׻ش��󣬲�������

�޷������������Զ�������


else{ //һֱδ���յ����ݡ�
		if(flag_heart_beart -- < -5000)
		{
						uCmd = USHIPGO;
							setBeepBlood = BCONTROL;	
							key3Cmd = 1;	
//			if(flag_heart_beart < -250)
//			printf("no heart flag_heart_beart = %d --------\r\n",flag_heart_beart);
//					Moto_Direction_Turn( MOTO_STOP, 0, 0);
		}

�޸��������޷�ֹͣ��״̬
							if(AS14B_receive_buf[8] == 1)
				{
						Moto_Direction_Turn( MOTO_STOP, 0, 0);
								key3Cmd = 0;
//						printf("stop---------\r\n");
				}

�޸�Ϊ�µ�ң�ؿ��Ʒ�ʽ

���ݱ��˵Ĵ�������״̬���޸ı���������ģʽ���ĸ���λ���ٶȿɵ���

����Ϊɾ����λ�ã����Ǵ���bug�ģ���
					if((AS14B_receive_buf[6]  <= BAR_CENTER_POSITION_MAX) &&((AS14B_receive_buf[7]  >= BAR_CENTER_POSITION_MIN)))//��������
					{
					
					}
						else if((AS14B_receive_buf[6]  >= BAR_X_CENTER_POSITION) &&((AS14B_receive_buf[7]  >= BAR_Y_CENTER_POSITION))) //��һ����
						{
														printf("��һ���� x  = %d, y=%d\r\n", AS14B_receive_buf[6] - 50 ,AS14B_receive_buf[7]-50);
							as14b_post_angle = (int) (atan2(AS14B_receive_buf[6]-50 , AS14B_receive_buf[7] -50) * 180/3.1416);
							as14b_post_length = (int)sqrt(AS14B_receive_buf[6]*AS14B_receive_buf[6] +AS14B_receive_buf[7] *AS14B_receive_buf[7]);
						as14b_post_length = 100;
							if(as14b_post_length > 100)
								as14b_post_length = 100;
							printf("��һ���� MOTO_LIGHT  pwm1  = %d, pwm2  = %d, as14b_post_angle =%d\r\n", as14b_post_length*1, abs(as14b_post_length * (sin(as14b_post_angle*3.1416/180))), as14b_post_angle);
								Moto_Direction_Turn(MOTO_LIGHT, as14b_post_length*1, abs(as14b_post_length * (sin(as14b_post_angle*3.1416/180))));
						}
						else 	if((AS14B_receive_buf[6]  >= BAR_X_CENTER_POSITION) &&((AS14B_receive_buf[7]  <= BAR_Y_CENTER_POSITION))) //��������
						{
								printf("�������� x  = %d, y=%d\r\n", AS14B_receive_buf[6] - 50 ,AS14B_receive_buf[7]);
					
									as14b_post_angle = (int) (atan2(AS14B_receive_buf[6] - 50 , AS14B_receive_buf[7] ) * 180/3.1416);
							
							as14b_post_length = (int)sqrt(AS14B_receive_buf[6]*AS14B_receive_buf[6] +AS14B_receive_buf[7] *AS14B_receive_buf[7]);
						
							as14b_post_length = 100;
							if(as14b_post_length > 100)
								as14b_post_length = 100;
								Moto_Direction_Turn(MOTO_LIGHT,  abs(as14b_post_length * (sin(as14b_post_angle*3.1416/180))), as14b_post_length*1);
	
								printf("�������� MOTO_LIGHT  pwm1  = %d, pwm2  = %d, as14b_post_angle =%d\r\n", abs(as14b_post_length * (sin(as14b_post_angle*3.1416/180))),  as14b_post_length*1, as14b_post_angle);
						}
						else 	if((AS14B_receive_buf[6]  <= BAR_X_CENTER_POSITION) &&((AS14B_receive_buf[7]  >= BAR_Y_CENTER_POSITION))) //�ڶ�����
						{
						
							as14b_post_angle = (int) (atan2(AS14B_receive_buf[6] , AS14B_receive_buf[7]) * 180/3.1416);
							as14b_post_length = (int)sqrt(AS14B_receive_buf[6]*AS14B_receive_buf[6] +AS14B_receive_buf[7] *AS14B_receive_buf[7]);
						
							if(as14b_post_length > 100)
								as14b_post_length = 100;
								Moto_Direction_Turn(MOTO_BACK, as14b_post_length*1, abs(as14b_post_length * (cos(as14b_post_angle*3.1416/180))));
							
							printf("�ڶ����� MOTO_BACK  pwm1  = %d, pwm2  = %d\r\n", as14b_post_length*1, abs(as14b_post_length * (cos(as14b_post_angle*3.1416/180))));
							
						}
						else 	if((AS14B_receive_buf[6]  <= BAR_X_CENTER_POSITION) &&((AS14B_receive_buf[7]  <= BAR_Y_CENTER_POSITION))) //��������
						{
							as14b_post_angle = (int) (atan2(AS14B_receive_buf[6] , AS14B_receive_buf[7]) * 180/3.1416);
							as14b_post_length = (int)sqrt(AS14B_receive_buf[6]*AS14B_receive_buf[6] +AS14B_receive_buf[7] *AS14B_receive_buf[7]);
						
							if(as14b_post_length > 100)
								as14b_post_length = 100;
								Moto_Direction_Turn(MOTO_BACK,  abs(as14b_post_length * (cos(as14b_post_angle*3.1416/180))), as14b_post_length*1 );
									printf("�������� MOTO_BACK  pwm1  = %d, pwm2  = %d\r\n", as14b_post_length*1, abs(as14b_post_length * (cos(as14b_post_angle*3.1416/180))));
							
						}
					


08 06 

114�汾
��OLED��ʾ�ľ�γ������ȥ����
����ص�ѹ�����ʾ��ADC1_10 * 11 

���Ӳ�����������ʾ������


GPS��EEPROM��Ҫ���ӣ�Ȼ���һ�����ԡ����ݵ���ʱ�䳤�˺����ǻᶪʧ���ݵ���Ϣ��

�޸Ŀ����ϵİ����޷�׼ȷֹͣ��ָ���жϲ�׼ȷ��ȱ������50��״̬�жϡ�






08 06 

113�汾

���������Ҫ�޸�Ϊʵʱ���ƣ����ܴ�����ʱ���������ԣ����ִ˴ε�mos����ȫ����ʵ��ʵʱ�ı�ռ�ձȼ����򣬶��Ҳ�����ַ��̵�����
�Ѿ�������뾫���޸�Ϊʵʱ���ġ�

08 06 

112�汾
�����߿����޸�Ϊҡ���·�ʽ���ơ����Ƿֳ��ĸ����ޣ����ݽǶ���ֵ����С��ֵ�ı�ռ�ձȡ���Ҫ���ù��ɶ���




08 05
111�汾

����led�Ŀ��ƣ����ú궨�巽ʽ��
����mos����������400ms�ıպϣ���ֹ��������̡�����ok��


08 05
110�汾

����adc�ɼ�����Ҫ��һ������λת������oled��ʾ��



08 04
109�汾

����Ϊ�µ�PCB��

OLED���������ϵ��޸ġ�

gpio_MSCK PC0 ---> 
gpio_MOSI PC1 ---> PC13 
gpio_DC   PD3 --->
gpio_RST  PD5 --->
gpio_CS   PD6 --->

	GPIOC->CRL&=0XFFFFFFF0; //PC0,1 OUT
	GPIOC->CRL|=0X00000003;	  
	GPIOC->ODR|=1<<0;
	GPIOC->CRH&=0XFF0FFFFF; //PC0,1 OUT
	GPIOC->CRH|=0X00300000;	  
 	GPIOC->ODR|=1<<13;
	
	#define OLED_SCLK PCout(0)
	#define OLED_SDIN PCout(13)

��ʾ��������ʾ


2016 -7 31
����ͨ��������˲��ֹͣ�����ǣ���Ҫͬʱ���ٵ�ĳ�ٶȣ�ĳ���򡣻���Ҫ�Ż���
����gps��У׼����У׼��ʼ��ת������ֹͣ��ת��
�յ㴦�Ĺ��䣬��Ҫ����ƫִ�Ƕȣ�������ص���ת������ת
��������gps�޸�Ϊ1Hz�����޸�Ϊ���з�ʽ��ð�ݡ��޸���5������ð�ݵı���Ԫ���ݡ�����GPS 6��7 ��û���⡣8 ���ն�����Щ���⡣
compass�Ķ�λ�̶�����Ҫ�Ȳ��Էֲ��Ƿ���ȣ�360�ȡ�Ȼ����һ�ξ��붨λ������Ҫд�ĵ���

108�汾��
---> �ڽ���107�汾���ӵ�����Ƶ�ƽ�����ƣ�˲��ֹͣ��
	case MOTO_STOP:
		TIM3->CCR3=100;
		TIM3->CCR4=100;
//		Moto_Right_Control(Moto_T, 0);
//		Moto_Left_Control(Moto_T, 0);


---> ���ӣ� gps��У׼����
		
		else if(rf_payload[6] == 6) //����
							{
								if((flag_calibrate_compass++%2) == 0)
								{
									USART_SendData(UART4,0X00);
									USART_SendData(UART4,0XC0); // У׼
								}
								else
								{
									USART_SendData(UART4,0X00);
									USART_SendData(UART4,0XC1); //ֹͣУ׼
								}
//							uCmd = USHIPBACK;
//							setBeepBlood = BSTART;	 //��������Ҫ�޸�״̬λ
							key3Cmd = 2;
							}


--->  �յ㴦�Ĺ��䣬��Ҫ����ƫִ�Ƕȣ�������ص���ת������ת����Ҫ���ԣ�10:28��

	if(angle_normal < angle_seek)
			{
				radian1 = abs(angle_normal + 360 - angle_seek);
				radian2 = abs(angle_seek - angle_normal);
				if(radian1 > 360)
					radian1 = radian1 - 360;
			}
			else
			{
				radian1 = abs(angle_normal  - angle_seek);
				radian2 = abs(angle_seek + 360 - angle_normal);
				if(radian2 > 360)
					radian2 = radian2 - 360;
			}
		
				pwm_offset = ((radian1  - radian2)/2) * pwm_p;
				
�����������ԣ�
��������gps�޸�Ϊ1Hz�����޸�Ϊ���з�ʽ��ð�ݡ�queue����δ�����ɹ�����Ҫ���ԡ�
�����飬��������
		

2016 -7 30

���������У���ԭ��ת��Ϊ����ת�������������޸ġ�
Moto_Direction_Turn(MOTO_RIGHT, 45, 65);

��������PID����ת����pid��Ҫ��������������ĿǰPΪ2 Dδ���ӡ�����Ŀǰ���ԡ�

��������gps�޸�Ϊ1Hz�����޸�Ϊ���з�ʽ��ð�ݡ�queue����δ�����ɹ�����Ҫ���ԡ�

2016 -7 24

ң�ؿ��ƹ����У���ԭ��ת��Ϊ����ת


2016 -7 23

�޸�beep��v����char���ţ�

��Ҫ��beep�����޸ģ������Ʋ��ֵĴ��롣

��ؿ���pwm������Ը���Ϊ0�����



2016 07 22
���ս��գ������쳣�޷���ȡgps����
�£������ǳ������޸Ĵ��ˣ����ޡ�
�£�����GPS���ݳ����쳣�����ޡ��²�������������⣬���ޡ�
�²⣬���ӿ��ܺ��������⣬�򵥿��Ǿ���Ӧ�ò��ǡ��󻻳�������Ϊ���˿����壬�Ѿ����ˡ�

5v�����λ�ã��ڷ����������ʱ�򣬻�����쳣gps���ǡ�

��ģ��󣬷�����Ȼ����ֶ��ǣ���������úܶࡣ�����ǵ�ص�ѹ���㣬��ʱ���ֵĵ�ѹ��10.������



2016 6 4

�޸��

�����Ƕȣ��޸�Ϊ3��
	if(((abs(angle_normal  - (Angle - 90)) < 3) ||((abs(abs(angle_normal - (Angle - 90) ) - 360) < 3))))
	
�޸�����270��-300�����ҷ�Χ��ʱ����޷�����ת��״̬
	
		if((Angle - 90) < 0)
		angle_2_oled = Angle + 360;
		OLED_ShowNum(92,36,(u32)(angle_2_oled - 90),6,12);


			angle_seek = Angle + 360;���Ƕ�С��0��ʱ����Ҫ����һ��360�ȡ�


�޸���λ�����ļ��㷽�����Ҹ�����Ϊ���Ƕȼ���Ӧ����Ŀǰλ�õ����������Ϊ׼�������λ�ã�ҲӦ����ˡ�







��������͵�ѹ



������ϵ�Դ���ѹѸ�ٱ���������ʲôԭ��?

������к��Դ��ѹѸ�ٱ������˿���ԭ���У�
��Դ������С��ɵ�ѹ������
��Դ�߹�С��ɵ�ѹ������
����������Դ�ߡ���Դ������ƥ����ɡ�
������ߴ�����ɡ�
��������ࡢ����ػ��Ѽ��ж�·����
������ع��صȵ���ɡ�


1�������·��2����Դ���ʲ�����3�������·�Ӵ�������

���Ŀ������12V��Դ��������

��Ҫ��pcb�塣pcb�����ӵػ���ɵ�ѹ���͡�������һ���ɵ��ڵ�Դ���ڵ���ϣ����ж��Ƿ��ǵ�Դ���⻹�ǵ�����⡣���Ų顣

������𶯵��������е�����N��


�綯��������ʱ���������Դﵽ�������3-7�������͵ĵ�������͸����ˣ�
������·�������е���ģ���ô��ĵ���ͨ����·���ͻ�����·�ϲ���һ����ѹ�������Ե���ϵĵ�ѹ�ͱ������ˡ���˴��͵ĵ��Ҫ���ý�ѹ������

�������ĵ�Դ�������·û���⣬�������Ϊ��Դ�Ĵ������������������µ�Դ��ѹ���������ˡ�
����Կ��ǻ�һ��������ʴ�ĵ�Դ���ԡ�

���������Ҳ�������������ˣ���������ˣ��ǵ�Դ��������������Ҫ����ѹ����Ҫ�п���Դ����������������



�������ݣ�
��һ�飺
new_dimensionality = 31.01614210;
new_longitude = 121.35121919;
old_dimensionality = 31.01612999;
old_longitude = 121.35142949;
2��
new_dimensionality = 31.01622150;
new_longitude = 121.35143770;
old_dimensionality = 31.01601209;
old_longitude = 121.35151149;
�Ƕ����3�ȣ�ʵ��343.117780��

3��
new_dimensionality =31.01601279 ;
new_longitude = 121.35152169;
old_dimensionality = 31.01559400;
old_longitude = 121.35153790;
ʵ�ʣ�358.090649��1�����
4��
new_dimensionality = 31.01607500;
new_longitude = 121.3531360;
old_dimensionality = 31.01652540;
old_longitude = 121.3530769;
ʵ�ʣ�173.552256��1�����




