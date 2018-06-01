﻿# 균형 로봇이란?  
스텝 모터로 바퀴를 일정한 각도만큼 돌려서 넘어지지 않도록하고 더 나아가서 로봇을 기울여서 앞으로 가고 좌우 회전등을 할 수 있도록 만든 로봇입니다.

## 한석민 선생님의 지원

<-- 이미지 추가 -->

이 프로젝트를 하는데 한석민 선생님의 로봇(하드웨어)를 빌려서 진행을 했습니다.

## 문제점과 해결 

 * __스텝 모터의 사용__

	스텝모터를 사용을 하는 것이 처음이라서 모터를 사용하는데 어려움이 많았습니다. 일반 DC 모터와는 다른 방식으로 엄청 신기하게 동작을 해서 처음엔 진행도 하지 못했지만 기본 원리 부터 이해를 하면서 모터의 사용이 쉬워졌습니다.
 
 * __하드웨어상의 문제__

 	이 로봇이 생각보다 부실해서 작년에 본격적으로 시작을 하지 못하는 이유도 하드웨어상의 문제였습니다. 그것을 다 해결을 하고 나서 본격적으로 만들어 보려고 하니 잘돌아가다가 갑자기 바퀴가 빠지고 그 바퀴를 적당히 쪼여주지 않으면 엄청 난 진동이 일어나고 그러니 모터가 발열이 나는 등의 문제가 있었습니다. 이러한 문제는 아직까지 진행을 못할 정도로 심각하게 작용하지는 않아서 다행이라고 생각하고 있습니다 ^^	
	
 * __균형을 잡기!__

	이것은 만들기 전부터 우려했던 문제였는데 아무것도 모르고 축 한개의 기울어지는 정도만 가지고 균형을 새우려고 노력을 했습니다. 하지만 이 방법은 엄청 노가다를 해서 값을 알아내야되고 계산을 통해서 완벽하다고 생각한 것도 실제로는 정상적이 동작이 힘들었습니다 그래서 선생님께 자문을 구해서  __PID 컨트롤__ 이라는 방법으로 균형을 잡으려고 도전하고 있습니다.

* __PID 컨트롤 공부__

 ![PID공식](/img/PID_note.jpg)
 

* __새로운 바디__

 ![김동균 선생님 로봇](/img/body_kdg.jpg)

한석민 선생님의 로봇으로 계속 프로젝트를 진행할 수 없을 만큼 부서져서 김동균 선생님의 로봇으로 프로젝트를 진행하게 되었습니다.

 # 여러가지 문제들
 
 * ## PID컨트롤 문제
 
 균형을 잡는 것을 잘 해내지 못해서 PID컨트롤을 공부하면서 자체적으로 PID 컨트롤 클래스를 만들어서 균형을 잡으려고 했으나... 
 
 제가 만든 클래스가 제대로 동학하지 않고 PID의 장점을 살리지 못하게 만들어져서 많은 문제가 있었습니다. 이러한 문제를 해결하기위해
 
 	* 다른 균형로봇의 코드를 본다
 
	* 처음부터 새로 PID 클래스를 만들어서 균형을 잡도록한다
 
	* 포기한다
 
 이러한 방법등이 있었는데 저는 다른 로봇의 코드를 보고 해석해서 제 로봇에 맞게 설개한 후 다시 짜는 방법으로 해결 하려고 했습니다.
 
 * ## 하드웨어 문제
 
 로봇을 만들면서 여러가지의 코드를 올려보고 보관을 하는 과정에 잘못 보관을 해서 3d프린터로 출력을  한 프레임이 뜯기고 바퀴 쪽이 부서지는 문제가
 있었습니다. 설상가상으로 제 로봇에 맞는 3d모델링 파일이 없어서 3d프린트로 뽑을 수 없는 상황에 오게 되었습니다. 
 
 저의 아두이노 프로젝트는 이 하드웨어의 문제로 꽤 오랜시간 동안 진행을 하지 못하고 있습니다.
