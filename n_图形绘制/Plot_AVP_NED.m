function Plot_AVP_NED(AVP0,AVP1)
% ������������е��Ż���
%
%   ��1����������������ݣ���2��Ϊ��������(��ɫ)..
%   ��2����������ĸ�ʽΪ(����) :
%           ʱ��(��λms) ��̬(x�����y������z���򣬵�λ����) ....
%           �ٶ�(x����y����z��) 
%           λ��(γ�ȡ����ȣ���λ���ȣ��߳� ��λ��)
%           λ��(�Ե�һ��Ϊԭ��ģ�����X������Y���߳�)
%

if nargin == 1
%% ������̬��Ϣ    
    figure;
    plot(AVP0(:,1),AVP0(:,2).*(180/pi));
    xlabel('\it t \rm / s');
    ylabel('\it \theta \rm / \circ');
    title('��̬-���');
    
    figure;
    plot(AVP0(:,1),AVP0(:,3).*(180/pi));
    xlabel('\it t \rm / s');
    ylabel('\it \gamma \rm / \circ');
    title('��̬-����');
    
    figure;
    plot(AVP0(:,1),AVP0(:,4).*(180/pi));
    xlabel('\it t \rm / s');
    ylabel('\it \psi \rm / \circ');
    title('��̬-����');
    
%% �����ٶ���Ϣ    
    figure;
    plot(AVP0(:,1),AVP0(:,5));
    xlabel('\it t \rm / s');
    ylabel('\it V_E \rm / m/s');
    title('�ٶ�-����');

    figure;
    plot(AVP0(:,1),AVP0(:,6));
    xlabel('\it t \rm / s');
    ylabel('\it V_N \rm / m/s');
    title('�ٶ�-����');

    figure;
    plot(AVP0(:,1),AVP0(:,7));
    xlabel('\it t \rm / s');
    ylabel('\it V_U \rm / m/s');
    title('�ٶ�-����');
    
    figure;
    plot(AVP0(:,1),(sqrt(AVP0(:,5).^2+AVP0(:,6).^2+AVP0(:,7).^2)));
    xlabel('\it t \rm / s');
    ylabel('\it V \rm / m/s');
    title('�����ٶ�');    

%% ����λ����Ϣ  
    figure;
    plot(AVP0(:,1),AVP0(:,8).*(180/pi));
    xlabel('\it t \rm / s');
    ylabel('\it γ�� \rm / \circ');
    title('λ��-γ��');
    
    figure;
    plot(AVP0(:,1),AVP0(:,9).*(180/pi));
    xlabel('\it t \rm / s');
    ylabel('\it ���� \rm / \circ');
    title('λ��-����');
    
    figure;
    plot(AVP0(:,1),AVP0(:,10));
    xlabel('\it t \rm / s');
    ylabel('\it �߳� \rm / m');
    title('λ��-�߳�');    

%% ���ƾ�γ����ʻ�켣 
    figure;
    plot(AVP0(1,9)*180/pi, AVP0(1,8)*180/pi, 'rp');     %����ʼλ�û�һ�� �����
    legend(sprintf('%.6f, %.6f / ��', AVP0(1,9)*180/pi,AVP0(1,8)*180/pi));
    hold on;    
    plot(AVP0(:,9).*(180/pi),AVP0(:,8).*(180/pi));
    xlabel('\it ���� \rm / circ');
    ylabel('\it γ�� \rm / circ');
    title('��ʻ·��(��γ��)');
    
%% ���� ������ ��ʻ�켣 
    %���������Ϊ���������ʻ�켣
    figure;
    plot( AVP0(1,12),  AVP0(1,11), 'rp');     %����ʼλ�û�һ�� �����
    legend(sprintf('��%.3f,  ��%.3f / ��', AVP0(1,11),  AVP0(1,12)));
    hold on;    
    plot(AVP0(:,12),AVP0(:,11));
    xlabel('\it ���� \rm / m');
    ylabel('\it ���� \rm / m');
    title('��ʻ·��');
    
end

if nargin == 2
%% ������̬��Ϣ    
    figure;
    plot(AVP0(:,1),AVP0(:,2).*(180/pi),'');
    hold on;
    plot(AVP1(:,1),AVP1(:,2).*(180/pi),'r');    
    xlabel('\it t \rm / s');
    ylabel('\it \theta \rm / \circ');
    title('��̬-���');
    
    figure;
    plot(AVP0(:,1),AVP0(:,3).*(180/pi),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,3).*(180/pi),'r');
    xlabel('\it t \rm / s');
    ylabel('\it \gamma \rm / \circ');
    title('��̬-����');
    
    figure;
    plot(AVP0(:,1),AVP0(:,4).*(180/pi),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,4).*(180/pi),'r');
    xlabel('\it t \rm / s');
    ylabel('\it \psi \rm / \circ');
    title('��̬-����');
    
%% �����ٶ���Ϣ    
    figure;
    plot(AVP0(:,1),AVP0(:,5),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,5),'r');
    xlabel('\it t \rm / s');
    ylabel('\it V_E \rm / m/s');
    title('�ٶ�-����');

    figure;
    plot(AVP0(:,1),AVP0(:,6),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,6),'r');
    xlabel('\it t \rm / s');
    ylabel('\it V_N \rm / m/s');
    title('�ٶ�-����');

    figure;
    plot(AVP0(:,1),AVP0(:,7),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,7),'r');
    xlabel('\it t \rm / s');
    ylabel('\it V_U \rm / m/s');
    title('�ٶ�-����');
    
    figure;
    plot(AVP0(:,1),(sqrt(AVP0(:,5).^2+AVP0(:,6).^2+AVP0(:,7).^2)),'r');
    hold on;
    plot(AVP1(:,1),(sqrt(AVP1(:,5).^2+AVP1(:,6).^2+AVP1(:,7).^2)));
    xlabel('\it t \rm / s');
    ylabel('\it V \rm / m/s');
    title('�����ٶ�');    

%% ����λ����Ϣ  
    figure;
    plot(AVP0(:,1),AVP0(:,8).*(180/pi),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,8).*(180/pi),'r');
    xlabel('\it t \rm / s');
    ylabel('\it γ�� \rm / \circ');
    title('λ��-γ��');
    
    figure;
    plot(AVP0(:,1),AVP0(:,9).*(180/pi),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,9).*(180/pi),'r');
    xlabel('\it t \rm / s');
    ylabel('\it ���� \rm / \circ');
    title('λ��-����');
    
    figure;
    plot(AVP0(:,1),AVP0(:,10),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,10),'r');
    xlabel('\it t \rm / s');
    ylabel('\it �߳� \rm / m');
    title('λ��-�߳�');    
    
    
    
   %% ���ƾ�γ����ʻ�켣 
    figure;
    plot(0, 0, 'rp');     %����ʼλ�û�һ�� �����
    legend(sprintf('%.6f, %.6f / ��', AVP0(1,9)*180/pi,AVP0(1,8)*180/pi));
    hold on;    
    plot(AVP0(:,9).*(180/pi),AVP0(:,8).*(180/pi),'b');
    hold on;    
    plot(AVP1(:,9).*(180/pi),AVP1(:,8).*(180/pi),'r');
    xlabel('\it ���� \rm / circ');
    ylabel('\it γ�� \rm / circ');
    title('��ʻ·��(��γ��)');
    
%% ���� ������ ��ʻ�켣 
    %���������Ϊ���������ʻ�켣
    figure;
    plot( AVP0(1,12),  AVP0(1,11), 'rp');     %����ʼλ�û�һ�� �����
    legend(sprintf('��%.3f,  ��%.3f / ��', AVP0(1,11),  AVP0(1,12)));
    hold on;    
    plot(AVP0(:,12),AVP0(:,11),'b');
    hold on;    
    plot(AVP1(:,12),AVP1(:,11),'r');
    xlabel('\it ���� \rm / m');
    ylabel('\it ���� \rm / m');
    title('��ʻ·��'); 
    
end    
