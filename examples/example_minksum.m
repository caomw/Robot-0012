A=5*[1 0; 0 1; -1 0; 0 -1]; 
%A=[1 0; 0 1;-1 -1; -1 0; 0 -1;2 -1]; 
%B=[3 3; 4 3; 4 4; 3 4]; 
B=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
[S,D]=minksum(-A,B); 
%S=S(D==1,:);
plot([B(1:end,1);B(1,1)],[B(1:end,2);B(1,2)],'-sr');
hold on
plot(A(:,1),A(:,2),'*',S(:,1),S(:,2),'d') 
k = boundary(S);
plot(S(k,1),S(k,2));
k=boundary(A);
plot(A(k,1),A(k,2));
%k=boundary(B);
%plot(B(k,1),B(k,2));
axis equal
grid on