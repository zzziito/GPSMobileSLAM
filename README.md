# 실외 환경을 위한 SLAM 및 Path Planning

이 프로젝트는 소프트웨어중심대학에서 진행하는 2022 봄학기 K-SW 스퀘어 프로그램에서 수행한 내용입니다. 

미국 퍼듀대학교에서 4명의 학생이 4개월간 진행했습니다😀

본 내용을 담은 [논문](https://drive.google.com/file/d/1XoPvBtw9vOMMglg9Vg_Dp4HWNM7q4Qec/view?usp=sharing)이 2022 IEEE IRC CHARMS 워크샵의 Regular Paper (8 pages) 로 승인되었습니다. 

## 결과 동영상

### GPS-Mobile SLAM

<img width="1342" alt="Screenshot 2022-11-18 at 8 35 31 PM" src="https://user-images.githubusercontent.com/52185595/202696546-9d7a9dca-521f-4655-96ab-c5f99b1c348e.png">
[https://youtu.be/2mR5yEIUdMo](https://youtu.be/2mR5yEIUdMo)

### 데모 차량 제작

![ezgif com-gif-maker](https://user-images.githubusercontent.com/52185595/202697340-7128bf4b-9954-4d7c-93c7-afd1b425d2e2.gif)


### Path Planning

<img src="%E1%84%89%E1%85%B5%E1%86%AF%E1%84%8B%E1%85%AC%20%E1%84%92%E1%85%AA%E1%86%AB%E1%84%80%E1%85%A7%E1%86%BC%E1%84%8B%E1%85%B3%E1%86%AF%20%E1%84%8B%E1%85%B1%E1%84%92%E1%85%A1%E1%86%AB%20SLAM%20%E1%84%86%E1%85%B5%E1%86%BE%20Path%20Planning%20f27a9fcb14274713939a75f7e556ce16/Screenshot_2022-11-13_at_2.08.53_PM.png"  width="600" height="450"/>


## 요약

<aside>
💡 이 프로젝트는 실외에서 진행되는 SLAM 과 Path Planning 에 관한 것입니다. “실외” 환경의 특성에 맞는 알고리즘을 개발하는 것이 프로젝트의 가장 중요한 부분이었습니다. 
Monocular 카메라를 이용한 ORB-SLAM2 의 모든 단계에 GPS 데이터를 추가하여 다양한 이점을 얻었고, Path Planning 의 경우 “도로의 안정성” 이라는 요소를 알고리즘에 추가하였습니다. 최적의 경로를 찾기 위해 안정적인 도로의 GPS 데이터에 큰 가중치를 부여하는 방식으로 시스템을 개선했습니다.

</aside>

## 프로젝트 동기

저희가 머물렀던 퍼듀는 인디애나 주에 있는 대학으로, 주변에는 농장이 많습니다. 대부분의 교수님들도 농장을 하나씩 가지고 계셔서 작물 재배와 농사 관련 기술에 대한 관심도가 높습니다. 

<img width="283" alt="Screenshot 2022-11-13 at 3 00 18 PM" src="https://user-images.githubusercontent.com/52185595/201508149-f99eb777-49cc-400e-84ab-fc1f0fe01a60.png">

인디애나 주의 농장

자재와 농기구를 운반해야 할 일이 많고 활동 범위가 넓어 자율 주행 기술에 대한 needs가 큰 곳입니다. 

자율주행을 위한 전역 경로 생성을 위해서는 지도가 필요합니다. 

실외 환경에서 주로 사용하는 지도는 GPS 를 사용해 제작하는 위성 지도입니다. 

실내 환경에서는 내부 구성을 자세하게 Mapping 하는 SLAM 기술을 사용합니다. 

<img width="300" alt="Screenshot 2022-11-13 at 3 01 53 PM" src="https://user-images.githubusercontent.com/52185595/201508193-6d0f023d-486c-4c80-9ee9-c5a9f489de74.png">


GPS 위성 지도


![ORB-SLAM 의 결과물 Point Cloud](%E1%84%89%E1%85%B5%E1%86%AF%E1%84%8B%E1%85%AC%20%E1%84%92%E1%85%AA%E1%86%AB%E1%84%80%E1%85%A7%E1%86%BC%E1%84%8B%E1%85%B3%E1%86%AF%20%E1%84%8B%E1%85%B1%E1%84%92%E1%85%A1%E1%86%AB%20SLAM%20%E1%84%86%E1%85%B5%E1%86%BE%20Path%20Planning%20f27a9fcb14274713939a75f7e556ce16/img7.jpg)

ORB-SLAM 의 결과물 Point Cloud

하지만 농장은 **실외이지만 실내의 특성을 가지고 있는 장소**라고 할 수 있습니다. 범위는 넓지만 GPS 로는 얻지 못하는 정보 (나무 둥치, 나뭇가지, 경사,  진흙 등) 들이 주행에 큰 영향을 미치기 때문입니다. 

![IMG_0772.jpg](%E1%84%89%E1%85%B5%E1%86%AF%E1%84%8B%E1%85%AC%20%E1%84%92%E1%85%AA%E1%86%AB%E1%84%80%E1%85%A7%E1%86%BC%E1%84%8B%E1%85%B3%E1%86%AF%20%E1%84%8B%E1%85%B1%E1%84%92%E1%85%A1%E1%86%AB%20SLAM%20%E1%84%86%E1%85%B5%E1%86%BE%20Path%20Planning%20f27a9fcb14274713939a75f7e556ce16/IMG_0772.jpg)

이와 같이 큰 나뭇가지가 있는 환경은 GPS 뿐만으로 알 수 없으므로 SLAM으로 자세히 매핑을 해 줄 필요가 있습니다. 또한 길을 찾을 때도 이와 같은 사항을 반영하여 길을 찾을 필요가 있습니다. 

## 프로젝트 목적

<aside>
💡 GPS 로 얻는 정보로는 충분하지 않은 상황에서 사용되는 
실외 SLAM 과 Path Planning 알고리즘 제작

</aside>

## 프로젝트 진행

### 실외 환경 특성

실외 환경은 실내와 비교하여 다음과 같은 특징이 있습니다. 
1) 교통 법규와 차선이 없다.
2) 장애물을 판단하는 기준이 필요하다.
3) 환경이 지속적으로 변한다.

이 특징들을 고려하며 다음 세 단계의 개발을 진행했습니다. 

아래 페이지에 각각의 알고리즘 개선에 대한 자세한 설명이 있습니다. 

[SLAM 알고리즘 설명](%E1%84%89%E1%85%B5%E1%86%AF%E1%84%8B%E1%85%AC%20%E1%84%92%E1%85%AA%E1%86%AB%E1%84%80%E1%85%A7%E1%86%BC%E1%84%8B%E1%85%B3%E1%86%AF%20%E1%84%8B%E1%85%B1%E1%84%92%E1%85%A1%E1%86%AB%20SLAM%20%E1%84%86%E1%85%B5%E1%86%BE%20Path%20Planning%20f27a9fcb14274713939a75f7e556ce16/SLAM%20%E1%84%8B%E1%85%A1%E1%86%AF%E1%84%80%E1%85%A9%E1%84%85%E1%85%B5%E1%84%8C%E1%85%B3%E1%86%B7%20%E1%84%89%E1%85%A5%E1%86%AF%E1%84%86%E1%85%A7%E1%86%BC%20387384e4a12f409893cbe791838ca726.md)

[Path Planning 알고리즘 설명](%E1%84%89%E1%85%B5%E1%86%AF%E1%84%8B%E1%85%AC%20%E1%84%92%E1%85%AA%E1%86%AB%E1%84%80%E1%85%A7%E1%86%BC%E1%84%8B%E1%85%B3%E1%86%AF%20%E1%84%8B%E1%85%B1%E1%84%92%E1%85%A1%E1%86%AB%20SLAM%20%E1%84%86%E1%85%B5%E1%86%BE%20Path%20Planning%20f27a9fcb14274713939a75f7e556ce16/Path%20Planning%20%E1%84%8B%E1%85%A1%E1%86%AF%E1%84%80%E1%85%A9%E1%84%85%E1%85%B5%E1%84%8C%E1%85%B3%E1%86%B7%20%E1%84%89%E1%85%A5%E1%86%AF%E1%84%86%E1%85%A7%E1%86%BC%2009c37cd8bef1442ea143c8cd6c6a6de7.md)

## 결과

### SLAM

<img width="391" alt="Screenshot 2022-11-13 at 12 56 52 PM" src="https://user-images.githubusercontent.com/52185595/201508261-72a5168e-2b1a-4791-b583-41c574d1d722.png">
<img width="393" alt="Screenshot 2022-11-13 at 12 57 31 PM" src="https://user-images.githubusercontent.com/52185595/201508264-58273c8d-5db5-4c28-8593-59a52075fe0c.png">

Table 1 에서, 기존의 ORB SLAM 보다 저희의 GPS Mobile SLAM 의 경우에 추출되는 Keyframe 과 Map Point 의 숫자가 현격하게 적은 것을 볼 수 있습니다. 

### 최적화 단계

![Screenshot 2022-11-13 at 1.08.19 PM.png](%E1%84%89%E1%85%B5%E1%86%AF%E1%84%8B%E1%85%AC%20%E1%84%92%E1%85%AA%E1%86%AB%E1%84%80%E1%85%A7%E1%86%BC%E1%84%8B%E1%85%B3%E1%86%AF%20%E1%84%8B%E1%85%B1%E1%84%92%E1%85%A1%E1%86%AB%20SLAM%20%E1%84%86%E1%85%B5%E1%86%BE%20Path%20Planning%20f27a9fcb14274713939a75f7e556ce16/Screenshot_2022-11-13_at_1.08.19_PM.png)

### 결과

기존의 ORB SLAM 으로 생성된 지도

![Untitled](%E1%84%89%E1%85%B5%E1%86%AF%E1%84%8B%E1%85%AC%20%E1%84%92%E1%85%AA%E1%86%AB%E1%84%80%E1%85%A7%E1%86%BC%E1%84%8B%E1%85%B3%E1%86%AF%20%E1%84%8B%E1%85%B1%E1%84%92%E1%85%A1%E1%86%AB%20SLAM%20%E1%84%86%E1%85%B5%E1%86%BE%20Path%20Planning%20f27a9fcb14274713939a75f7e556ce16/Untitled.png)

우리의 알고리즘으로 만든 지도

![Untitled](%E1%84%89%E1%85%B5%E1%86%AF%E1%84%8B%E1%85%AC%20%E1%84%92%E1%85%AA%E1%86%AB%E1%84%80%E1%85%A7%E1%86%BC%E1%84%8B%E1%85%B3%E1%86%AF%20%E1%84%8B%E1%85%B1%E1%84%92%E1%85%A1%E1%86%AB%20SLAM%20%E1%84%86%E1%85%B5%E1%86%BE%20Path%20Planning%20f27a9fcb14274713939a75f7e556ce16/Untitled%201.png)

### 알고리즘 개선 후

저희는 이 세 과정에 GPS 데이터를 추가함으로써, 다음과 같은 이점을 얻을 수 있었습니다.

1. **의미 없는 연산을 방지함으로써 알고리즘의 Time Complexity 감소**

실외 주행 특성상 차량에게 이동 명령을 내려도 의도한 만큼 차량이 움직이지 않는 경우가 잦습니다. (진흙으로 인해 바퀴가 헛도는 현상 등) 
따라서 GPS 데이터를 이용해 차량의 이동 거리를 계산하여 만약 이동 거리가 유의미하지 않다면 Tracking 과정을 수행하지 않도록 하였습니다. 
또한 무의미한 KeyFrame 은 오히려 전체 지도의 정확도를 떨어뜨리기 때문에, KeyFrame 후보군 (Candidate KeyFrame) 과 기존 KeyFrame 사이의 거리가 일정 거리 이하일 때는 Map 에 추가하지 않도록 개발하였습니다. 
이를 통해 지도의 정확성을 높이고 연산량을 줄였습니다.

2. **Covisibility Graph 제작 과정에서의 정확도 향상**

새로운 KeyFrame 이 추가될 때, 동일한 위치의 지도를 공유하는 다른 KeyFrame 들의 Covisibility Graph 가 업데이트됩니다. 
이 그래프를 이용하여 모든 KeyFrame 을 비교하지 않고도 시스템은 차량의 자세를 추정할 수 있습니다. 이 그래프를 만들 때, KeyFrame 들의 Correlation 을 추정하기 위해 화면의 특징점들이 이용됩니다. 
하지만 실외 환경에서는 (특히 숲 지형에서) 실내보다 특징점을 추출하기 어렵기 때문에, 이 과정에서 GPS 데이터를 추가하여 KeyFrame 간의 거리를 비교하는 방식으로 알고리즘을 개선했습니다.

3. **Relocalization 정확도 향상**

차량이 Tracking 과정을 정상적으로 실행하지 못 했다면, 이전의 데이터를 이용해서 현재 위치를 추정해야 합니다. 
이 과정에서 KeyFrame Database 는 BOW (Bag Of Words) 를 이용해서 현재 Frame 과 유사도가 높은 KeyFrame 을 찾습니다. 하지만 이 과정은 많은 컴퓨팅 파워를 소모하고, 시간이 오래 걸립니다. 또한 KeyFrame Database 만 이용하는 것은 신뢰도가 높지 않습니다. 따라서 이 과정에도 GPS 데이터를 추가했습니다. 먼저, 현재 위치에서 먼 곳에 있는 KeyFrame 은 연산에서 제외했습니다. 이렇게 추려진 candidate KeyFrame 중 현재 위치와 비슷한 GPS 데이터를 가지고 있는 KeyFrame 만 연산하여, 
시간 복잡도를 줄이고 결정의 신뢰도를 높였습니다.

### Path Planning

![Screenshot 2022-11-13 at 1.22.22 PM.png](%E1%84%89%E1%85%B5%E1%86%AF%E1%84%8B%E1%85%AC%20%E1%84%92%E1%85%AA%E1%86%AB%E1%84%80%E1%85%A7%E1%86%BC%E1%84%8B%E1%85%B3%E1%86%AF%20%E1%84%8B%E1%85%B1%E1%84%92%E1%85%A1%E1%86%AB%20SLAM%20%E1%84%86%E1%85%B5%E1%86%BE%20Path%20Planning%20f27a9fcb14274713939a75f7e556ce16/Screenshot_2022-11-13_at_1.22.22_PM.png)

기존의 BIT* 알고리즘은 회색 영역 (불안정한 길) 상관 없이 최단 길이만 고려해 길을 찾지만, 개선된 알고리즘의 경우 흰색 영역 (안정한 길) 을 중심으로 길을 찾아 가는 것을 볼 수 있습니다. 


<img width="416" alt="Screenshot 2022-11-13 at 1 25 08 PM" src="https://user-images.githubusercontent.com/52185595/201508275-d8b5cf96-8deb-4510-a3b9-94a9d2c8f5f9.png">
<img width="530" alt="Screenshot 2022-11-13 at 1 25 04 PM" src="https://user-images.githubusercontent.com/52185595/201508277-fc59c870-2b15-4559-952d-77e3843f81f4.png">

주변 장애물과의 거리도 기존 알고리즘보다 먼 것으로 측정됩니다. 

또한 샘플의 개수도 더 적어 알고리즘 Time Complexity 를 줄일 수 있었습니다. 

## 개발자들 👥

<img width="400" alt="Screenshot 2022-11-13 at 12 56 52 PM" src="https://user-images.githubusercontent.com/52185595/201508998-4a209b03-0420-4616-b48e-6563fb99a18c.jpg">

허성일 `tjddlf101@hufs.ac.kr`

문주은 `cindy4741@khu.ac.kr`

최지웅 `jiwung22@gmail.com`

박지원 `overflow21@khu.ac.kr`


