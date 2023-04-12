avmean=[];
nvmean=[];
avvmean=zeros(21);
nvvmean=zeros(21);
totalav=[];
totalnv=[];
totalavv=zeros(21);
totalnvv=zeros(21);
M = 216;
N = 216;
T=30;
G1 = [1	2	3	4	4	4	4	5	5	6	6	6	6	7	8	8	9	9	10	11	11	11	11	12	12	13	13	13	13	14	15	16];
G2 = [4	6	4	1	3	5	8	4	6	2	5	7	9	6	4	11	6	13	11	8	10	12	15	11	13	9	12	14	16	13	11	13];
G3 = [50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50	50];
G = digraph(G1,G2,G3);
A=[4    3	1	5	8
6   5   2   7   9
11  10  8   12  15
13  12  9   14  16
5   4   0   6   0
8   0   4   0   11
9   0   6   0   13
12  11  0   13  0];
SS=[1   0   22  N
    2   0   19  N
    3   0   1   1
    7   10  0   M
    10  0   4   1
    14  7   0   M
    15  13  0   1
    16  16  0   1
    5   10  1   108
    8   13  22  108
    9   16  19  108
    12  7   4   108
    ];
roads=[1	166	0   7
1	165	1   7
1	164	2   7
1	58	0   14
1	57	1   14
1	56	2   14
2	51	0   10
2	52	1   10
2	53	2   10
2	159	0   3
2	160	1   3
2	161	2   3
3	51	0   1
3	52	1   1
3	53	2   1
3	159	0   2
3	160	1   2
3	161	2   2
4	166	0   16
4	165	1   16
4	164	2   16
4	58	0   15
4	57	1   15
4	56	2   15
    ];
%dir hor ver1 ver2 node cycle stop go
pt=[								
1	50	164	166	4	4*T	T-10	0
2	59	159	161	4	4*T	3*T-10	2*T
3	158	51	53	4	4*T	2*T-10	T
4	167	56	58	4	4*T	4*T-10	3*T
1	158	164	166	6	4*T	T-10	0
2	167	159	161	6	4*T	3*T-10	2*T
3	158	159	161	6	4*T	2*T-10	T
4	167	164	166	6	4*T	4*T-10	3*T
1	50	56	58	11	4*T	T-10	0
2	59	51	53	11	4*T	3*T-10	2*T
3	50	51	53	11	4*T	2*T-10	T
4	59	56	58	11	4*T	4*T-10	3*T
1	158	56	58	13	4*T	T-10	0
2	167	51	53	13	4*T	3*T-10	2*T
3	50	159	161	13	4*T	2*T-10	T
4	59	164	166	13	4*T	4*T-10	3*T
1	108	164	166	5	0	0	0
2	108	159	161	5	0	0	0
3	108	51	53	8	0	0	0
4	108	56	58	8	0	0	0
1	108	56	58	12	0	0	0
2	108	51	53	12	0	0	0
3	108	159	161	9	0	0	0
4	108	164	166	9	0	0	0
];
spathdata=cell(16);
for simno=1:150
    simno
clearvars -except time simno avmean nvmean avvmean nvvmean totalav totalnv totalavv totalnvv G1 G2 G3 G A SS roads pt M N T spathdata
l=0.02%example of x parameter->mod(floor((simno-1)/3),10)/50;
lambda=[l
    l
    l
    l
    l
    l
    l
    l
    l
    l
    l
    l
];
pauto=0.5;
plocate=1;%example of y parameter->floor((simno-1)/30)/4;
showgraph=0;
delay=0;
data_step=120;
C=zeros(M+1,N+1);
%debug1=C;
%debug2=debug1;
%debug3=debug2;
stepmax = 1000;
vmax=5; %14.8 m/s=53.28km/h   
x=[];%xpos
y=[];%ypos
d=[];%dir
l=[];%lane
c=[];%completion
j=[];
vchange=[];
v=[];%velocity
vstep_1=[];
singleturn=[];
st=[];%brake
rt=[];%turn
lt=[];
vehin=[];%vehicles in system
k=[];%k-density
AV=[];
NV=[];
CAV=[];
CNV=[];
S=[];
E=[];
EX=[];
a_1=[];
node=[];%location of car within node
path={};
id=[];%path id
auto=[];
ll=[];

travelvs=zeros(data_step,numel(G3));
travelnos=zeros(data_step,numel(G3));
for a=1:data_step
    for b=1:numel(G3)
        travelvs(a,b)=nan;
        travelnos(a,b)=nan;
    end
end
nosd=0;
at=zeros(1,numel(SS)/4);
dat=zeros(1,numel(SS)/4);
for o=randperm(numel(SS)/4)
    while dat(o)==SS(o,1) || dat(o)==0
        dat(o)=SS(random('unid',numel(SS)/4),1);
    end
end
vehcount=0;
ccount=zeros(numel(roads)/3,1);
sig=[1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,1,1,1,1,1,1,1];
 step=0; %t is the number of steps
 %START
 
while step<stepmax
    %debug3=debug2;
    %debug2=debug1;
    %debug1=C;
    pause(delay);
    G4=zeros(1,numel(G3));
    G5=zeros(1,numel(G3));
    recv=[];
    for n=vehin
        if rand<plocate
            ee=findedge(G,path{n}(id(n)),path{n}(1+id(n)));
            recv=[recv;ee,v(n)];
        end
    end
  
    if ~isempty(recv)
        for ee=1:numel(G3)
            travelvs(data_step+1,ee)=nansum(recv(recv(:,1)==ee,2));
            travelnos(data_step+1,ee)=numel(recv(recv(:,1)==ee,2));
        end
            travelvs(1,:)=[];
            travelnos(1,:)=[];
    end
    for ee=1:numel(G3)
        if ~isnan(nansum(travelvs(:,ee))/nansum(travelnos(:,ee)))
            G6(ee)=G3(ee)/(nansum(travelvs(:,ee))/nansum(travelnos(:,ee)));
        else
            G6(ee)=42.08*(nnz(G2(ee)==[4,6,11,13])~=0)+G3(ee)/vmax;
        end
    end
    timeG=digraph(G1,G2,G6);

        for o=randperm(numel(SS)/4)
            if step>=at(o)
                
                ori=SS(o,1);
                dest=dat(o);
                vehcount=vehcount+1;
                if rand<pauto
                    auto=[auto, vehcount];
                    path=[path;shortestpath(timeG,ori,dest)];
                else
                    if isempty(spathdata{ori,dest})
                    spathdata{ori,dest}=shortestpath(G,ori,dest);
                    end
                    path=[path;spathdata{ori,dest}];
                end
                if path{end}(2)<ori
                    L=SS(SS(:,1)==ori,2);
                else
                    L=SS(SS(:,1)==ori,3);
                end
                if length(path{end})>2
                    L=3*floor((L-1)/3)+mod(find(path{end}(3)==A(path{end}(2) ...
                    ==A(:,1),:))-find(path{end}(1)==A(path{end}(2)==A(:,1),:)),4);
                end

                D=roads(L,1);
                P=roads(L,2);
                Q=SS(SS(:,1)==ori,4);
                if D<3 && C(Q,P)==0 || D>2 && C(P,Q)==0
                    dat(o)=0;
                    while dat(o)==ori || dat(o)==0
                        dat(o)=SS(random('unid',numel(SS)/4),1);
                    end
                    at(o)=find(1-exp(-(1:4000)*lambda(o))>=rand,1)+step;
                    ccount(L)=ccount(L)+1;
                    vehin=[vehin,vehcount];
                    id=[id,1];
                    v=[v,vmax];%#ok<*AGROW>
                    st=[st,0];
                    rt=[rt,0];
                    lt=[lt,0];
                    d=[d,D];
                    l=[l,L];
                    c=[c,0];
                    j=[j,0];
                    vstep_1=[vstep_1,0];
                    vchange=[vchange,0];
                    a_1=[a_1,0];
                    node=[node,0];
                    singleturn=[singleturn,0];
                    ll=[ll,L];
                    if D<3
                        x=[x,Q];
                        y=[y,P];
                        C(Q,P)=vehcount;
                    else
                        x=[x,P];
                        y=[y,Q];
                        C(P,Q)=vehcount;
                    end
                else
                    %CANCEL
                    at(o)=1+step;
                    if ~isempty(auto) && auto(end)==vehcount
                        auto(end)=[];
                    end
                    vehcount=vehcount-1;
                    path(end)=[];
                end
            end
        end
    %signal changing
    for p=1:numel(pt)/8
        if mod(step,pt(p,6))==pt(p,7)
            sig(p)=0;
        end
        if mod(step,pt(p,6))==pt(p,8)
            sig(p)=1;
        end
    end
for n = vehin
    vstep_1(n)=v(n);
    if v(n)<0 || v(n)>vmax
        warning('Car %d is out of speed limit.',n)
    end
    if id(n)+1<length(path{n})
        %if the next node is not a junction, skip it
        if nnz(path{n}(1+id(n))==[5,8,9,12])==0 %SS(:,1)
                ll(n)=3*floor((l(n)-1)/3)+mod(find(path{n}(2+id(n)) ...
                ==A(path{n}(1+id(n))==A(:,1),:))-find(path{n}(id(n))==A(path{n}(1+id(n))==A(:,1),:)),4);
        else
            if id(n)+2<length(path{n})
                ll(n)=3*floor((l(n)-1)/3)+mod(find(path{n}(3+id(n)) ...
                ==A(path{n}(2+id(n))==A(:,1),:))-find(path{n}(1+id(n))==A(path{n}(2+id(n))==A(:,1),:)),4);
            else
                ll(n)=l(n);
            end
        end
    else
        ll(n)=l(n);
    end
end
for n = vehin
    if v(n)<vmax
        v(n)=v(n)+1;
    end
    %change lane
    if node(n)==0
        if d(n)<3
            if C(x(n),roads(ll(n),2))==0
                    C(x(n),y(n))=0;
                    y(n)=roads(ll(n),2); %#ok<*SAGROW>
                    l(n)=ll(n);
                    C(x(n),y(n))=n;
                    %reset as lane has changed
                    st(n)=0;
                    rt(n)=0;
                    lt(n)=0;
            elseif ll(n)~=l(n) && v(n)>0 || ll(C(x(n),roads(ll(n),2)))==l(n) && ll(n)<l(n) && v(n)>0
                v(n)=v(n)-1;
            end
        else
            if C(roads(ll(n),2),y(n))==0
                    C(x(n),y(n))=0;
                    x(n)=roads(ll(n),2);
                    l(n)=ll(n);
                    C(x(n),y(n))=n;
                    %reset as lane has changed
                    st(n)=0;
                    rt(n)=0;
                    lt(n)=0;
            elseif ll(n)~=l(n) && v(n)>0 || ll(C(roads(ll(n),2),y(n)))==l(n) && ll(n)<l(n) && v(n)>0
                v(n)=v(n)-1;
            end
        end
    end
end
for n = vehin
    %change the direction
    %signal check
    for p=1:numel(pt)/8
        if pt(p,1)==d(n) %signal serves same dir as car
            if sig(p)==0 %&& roads(l(a),3)~=0 %stop at signal except for left turns
                if v(n)>=pt(p,2)-x(n) && pt(p,2)-x(n)>=0 && nnz(y(n)==pt(p,3):pt(p,4))>0 && d(n)==1||...
                   v(n)>=x(n)-pt(p,2) && x(n)-pt(p,2)>=0 && nnz(y(n)==pt(p,3):pt(p,4))>0 && d(n)==2||...
                   v(n)>=pt(p,2)-y(n) && pt(p,2)-y(n)>=0 && nnz(x(n)==pt(p,3):pt(p,4))>0 && d(n)==3||...
                   v(n)>=y(n)-pt(p,2) && y(n)-pt(p,2)>=0 && nnz(x(n)==pt(p,3):pt(p,4))>0 && d(n)==4
                    st(n)=p;
                end
            end
            
            if x(n)-pt(p,2)>=1 && x(n)-pt(p,2)<=8 && nnz(y(n)==pt(p,3):pt(p,4))>0 && d(n)==1 || ...
               pt(p,2)-x(n)>=1 && pt(p,2)-x(n)<=8 && nnz(y(n)==pt(p,3):pt(p,4))>0 && d(n)==2 || ...
               y(n)-pt(p,2)>=1 && y(n)-pt(p,2)<=8 && nnz(x(n)==pt(p,3):pt(p,4))>0 && d(n)==3 || ...
               pt(p,2)-y(n)>=1 && pt(p,2)-y(n)<=8 && nnz(x(n)==pt(p,3):pt(p,4))>0 && d(n)==4
                node(n)=pt(p,5);
                if node(n)==path{n}(end)
                    vehin=vehin(vehin~=n);
                    C(x(n),y(n))=0;
                    c(n)=1;
                    ex=ex+1;
                end
                if nnz(node(n)==path{n})==0
                    warning('Car %d went to %d instead of %s', n, node(n), num2str(path{n}))
                end
                if node(n)==path{n}(id(n)+1)
                    id(n)=id(n)+1;
                end
            end
            %exiting node
            if x(n)-pt(p,2)>=9 && x(n)-pt(p,2)<=9+vmax && nnz(y(n)==pt(p,3):pt(p,4))>0 && d(n)==1 || ...
               pt(p,2)-x(n)>=9 && pt(p,2)-x(n)<=9+vmax && nnz(y(n)==pt(p,3):pt(p,4))>0 && d(n)==2 || ...
               y(n)-pt(p,2)>=9 && y(n)-pt(p,2)<=9+vmax && nnz(x(n)==pt(p,3):pt(p,4))>0 && d(n)==3 || ...
               pt(p,2)-y(n)>=9 && pt(p,2)-y(n)<=9+vmax && nnz(x(n)==pt(p,3):pt(p,4))>0 && d(n)==4
                node(n)=pt(p,5);
                if node(n)==path{n}(id(n)+1)

                    id(n)=id(n)+1;
                end
                node(n)=0;
                singleturn(n)=0;
            end
            if p<17
                if roads(l(n),3)==2 && st(n)==0 && singleturn(n)==0%right turning vehicles
                    if 6+pt(p,2)-x(n)<=v(n) && 6+pt(p,2)-x(n)>0 && nnz(y(n)==pt(p,3):pt(p,4))>0 && d(n)==1 || ...
                       6+x(n)-pt(p,2)<=v(n) && 6+x(n)-pt(p,2)>0 && nnz(y(n)==pt(p,3):pt(p,4))>0 && d(n)==2 || ...
                       6+pt(p,2)-y(n)<=v(n) && 6+pt(p,2)-y(n)>0 && nnz(x(n)==pt(p,3):pt(p,4))>0 && d(n)==3 || ...
                       6+y(n)-pt(p,2)<=v(n) && 6+y(n)-pt(p,2)>0 && nnz(x(n)==pt(p,3):pt(p,4))>0 && d(n)==4
                        rt(n)=p;
                        singleturn(n)=1;
                    end
                elseif roads(l(n),3)==0 && st(n)==0 %left turning vehicles
                    if 1+pt(p,2)-x(n)<=v(n) && 1+pt(p,2)-x(n)>0 && nnz(y(n)==pt(p,3):pt(p,4))>0 && d(n)==1 || ...
                       1+x(n)-pt(p,2)<=v(n) && 1+x(n)-pt(p,2)>0 && nnz(y(n)==pt(p,3):pt(p,4))>0 && d(n)==2 || ...
                       1+pt(p,2)-y(n)<=v(n) && 1+pt(p,2)-y(n)>0 && nnz(x(n)==pt(p,3):pt(p,4))>0 && d(n)==3 || ...
                       1+y(n)-pt(p,2)<=v(n) && 1+y(n)-pt(p,2)>0 && nnz(x(n)==pt(p,3):pt(p,4))>0 && d(n)==4
                        lt(n)=p;
                    end
                end
            end
        end
    end
    if st(n)~=0
        p=st(n);
        if d(n)==1
            v(n)=pt(p,2)-x(n);
        elseif d(n)==2
            v(n)=x(n)-pt(p,2);
        elseif d(n)==3
            v(n)=pt(p,2)-y(n);
        else
            v(n)=y(n)-pt(p,2);
        end
    end
end
for n=vehin
    j(n)=inf;
    %finding cars
    %nos = next occupied site
    if d(n)==1
        nos=find(C(x(n)+1:M,y(n))~=0,1)+x(n);
    elseif d(n)==2
        nos=find(C(1:x(n)-1,y(n))~=0,1,'last');
    elseif d(n)==3
        nos=find(C(x(n),y(n)+1:N)~=0,1)+y(n);
    else
        nos=find(C(x(n),1:y(n)-1)~=0,1,'last');
    end
    if ~isempty(nos)
        if d(n)<3
            j(n)=abs(nos-x(n));
        else
            j(n)=abs(nos-y(n));
        end
    end
    if rt(n)~=0
        p=rt(n);
        if d(n)==1
            if isempty(nos) || nos>pt(p,2)+6
            nosd=1;
            nos=find(C(6+pt(p,2),1:y(n))~=0,1,'last');
            if ~isempty(nos)
            j(n)=y(n)-nos+pt(p,2)+6-x(n);
            end
            end
        elseif d(n)==2
            if isempty(nos) || nos<pt(p,2)-6
            nosd=1;
            nos=find(C(pt(p,2)-6,y(n):N)~=0,1)+y(n)-1;
            if ~isempty(nos)
            j(n)=nos-y(n)+x(n)-pt(p,2)+6;
            end
            end
        elseif d(n)==3
            if isempty(nos) || nos>pt(p,2)+6
            nosd=1;
            nos=find(C(x(n):M,6+pt(p,2))~=0,1)+x(n)-1;
            if ~isempty(nos)
            j(n)=nos-x(n)+pt(p,2)+6-y(n);
            end
            end
        else
            if isempty(nos) ||  nos<pt(p,2)-6
            nosd=1;
            nos=find(C(1:x(n),pt(p,2)-6)~=0,1,'last');
            if ~isempty(nos)
            j(n)=x(n)-nos+y(n)-pt(p,2)+6;
            end
            end
        end
    elseif lt(n)~=0
        p=lt(n);
        if d(n)==1
            if isempty(nos) || nos>pt(p,2)+1
            nosd=1;
            nos=find(C(pt(p,2)+1,y(n):N)~=0,1)+y(n)-1;
            if ~isempty(nos)
            j(n)=nos-y(n)+pt(p,2)+1-x(n);
            end
            end
        elseif d(n)==2
            if isempty(nos) || nos<pt(p,2)-1
            nosd=1;
            nos=find(C(pt(p,2)-1,1:y(n))~=0,1,'last');
            if ~isempty(nos)
            j(n)=y(n)-nos+x(n)-pt(p,2)+1;
            end
            end
        elseif d(n)==3
            if isempty(nos) || nos>pt(p,2)+1
            nosd=1;
            nos=find(C(1:x(n),pt(p,2)+1)~=0,1,'last');
            if ~isempty(nos)
            j(n)=x(n)-nos+pt(p,2)+1-y(n);
            end
            end
        else
            if isempty(nos) || nos<pt(p,2)-1
            nosd=1;
            nos=find(C(x(n):M,pt(p,2)-1)~=0,1)+x(n)-1;
            if ~isempty(nos)
            j(n)=nos-x(n)+y(n)-pt(p,2)+1;
            end
            end
        end
    end
    if ~isempty(nos) && nnz(auto==n)==1 && ...
       (nnz(C(nos,y(n))==auto)~=0 && d(n)<3 || nnz(C(x(n),nos)==auto)~=0 && d(n)>2) %both are AVs
           if nosd==1
               if d(n)==1
                   if rt(n)~=0
                       a_1(n)=C(pt(p,2)+6,nos);
                   else
                       a_1(n)=C(pt(p,2)+1,nos);
                   end
               elseif d(n)==2
                   if rt(n)~=0
                       a_1(n)=C(pt(p,2)-6,nos);
                   else
                       a_1(n)=C(pt(p,2)-1,nos);
                   end
               elseif d(n)==3
                   if rt(n)~=0
                       a_1(n)=C(nos,pt(p,2)+6);
                   else
                       a_1(n)=C(nos,pt(p,2)+1);
                   end
               else
                   if rt(n)~=0
                       a_1(n)=C(nos,pt(p,2)-6);
                   else
                       a_1(n)=C(nos,pt(p,2)-1);
                   end
               end
           else
               if d(n)<3
                   a_1(n)=C(nos,y(n));
               else
                   a_1(n)=C(x(n),nos);
               end
           end
    end
    nosd=0;   
end
for n=vehin
    vchange(n)=nan;
end
while nnz(isnan(vchange))~=0
    for n=vehin
        if isnan(vchange(n)) 
            if a_1(n)~=0
                if ~isnan(vchange(a_1(n)))
                    if st(n)~=0 && v(a_1(n))+j(n)<=v(n)+1 || st(n)==0 && v(a_1(n))+j(n)<=vstep_1(n)+2
                       v(n)=min([vmax,v(a_1(n))+j(n)-1]);
                    end
                    a_1(n)=0;
                    vchange(n)=v(n)-vstep_1(n);
                    st(n)=0;
                end
            else
                %deceleration due to nos
                if j(n)<=v(n)
                    v(n)=j(n)-1;
                end
                %RANDOMIZATION
                if rand<0.5 && v(n)>0 && nnz(auto==n)==0
                    v(n)=v(n)-1;
                end
                vchange(n)=v(n)-vstep_1(n);
                st(n)=0;
            end
        end
    end
end
%clear old sites
ex=0;
C=zeros(M+1,N+1);
for n=vehin
    %update new pos and sites
    if d(n)==1
        if rt(n)~=0 && 6+pt(rt(n),2)-x(n)<=v(n)
            p=rt(n);
            y(n)=y(n)-v(n)+pt(p,2)+6-x(n);
            x(n)=pt(p,2)+6;
            d(n)=4;
            rt(n)=0;
            l(n)=find(roads(:,1)==4 & roads(:,2)==x(n));
        elseif lt(n)~=0 && 1+pt(lt(n),2)-x(n)<=v(n)
            p=lt(n);
            y(n)=y(n)+v(n)-pt(p,2)-1+x(n);
            x(n)=pt(p,2)+1;
            d(n)=3;
            lt(n)=0;
            l(n)=find(roads(:,1)==3 & roads(:,2)==x(n));
        else
        x(n)=x(n)+v(n);
        end
    elseif d(n)==2
        if rt(n)~=0 && 6+x(n)-pt(rt(n),2)<=v(n)
            p=rt(n);
            y(n)=y(n)+v(n)-x(n)+pt(p,2)-6;
            x(n)=pt(p,2)-6;
            d(n)=3;
            rt(n)=0;
            l(n)=find(roads(:,1)==3 & roads(:,2)==x(n));
        elseif lt(n)~=0 && 1+x(n)-pt(lt(n),2)<=v(n)
            p=lt(n);
            y(n)=y(n)-v(n)+x(n)-pt(p,2)+1;
            x(n)=pt(p,2)-1;
            d(n)=4;
            lt(n)=0;
            l(n)=find(roads(:,1)==4 & roads(:,2)==x(n));
        else
        x(n)=x(n)-v(n);
        end
    elseif d(n)==3
        if rt(n)~=0 && 6+pt(rt(n),2)-y(n)<=v(n)
            p=rt(n);
            x(n)=x(n)+v(n)-pt(p,2)-6+y(n);
            y(n)=pt(p,2)+6;
            d(n)=1;
            rt(n)=0;
            l(n)=find(roads(:,1)==1 & roads(:,2)==y(n));
        elseif lt(n)~=0 && 1+pt(lt(n),2)-y(n)<=v(n)
            p=lt(n);
            x(n)=x(n)-v(n)+pt(p,2)+1-y(n);
            y(n)=pt(p,2)+1;
            d(n)=2;
            lt(n)=0;
            l(n)=find(roads(:,1)==2 & roads(:,2)==y(n));
        else
        y(n)=y(n)+v(n);
        end
    else
        if rt(n)~=0 && 6+y(n)-pt(rt(n),2)<=v(n)
            p=rt(n);
            x(n)=x(n)-v(n)+y(n)-pt(p,2)+6;
            y(n)=pt(p,2)-6;
            d(n)=2;
            rt(n)=0;
            l(n)=find(roads(:,1)==2 & roads(:,2)==y(n));
        elseif lt(n)~=0 && 1+y(n)-pt(lt(n),2)<=v(n)
            p=lt(n);
            x(n)=x(n)+v(n)-y(n)+pt(p,2)-1;
            y(n)=pt(p,2)-1;
            d(n)=1;
            lt(n)=0;
            l(n)=find(roads(:,1)==1 & roads(:,2)==y(n));
        else
        y(n)=y(n)-v(n);
        end
    end
    if x(n)>M && d(n)==1 || x(n)<1 && d(n)==2 || y(n)>N && d(n)==3 || y(n)<1 && d(n)==4
        %complete
        if roads(l(n),4)~=path{n}(end)
            warning('Car %d went to %d instead of %s', n, roads(l(n),4), num2str(path{n}))
        end
        vehin=vehin(vehin~=n);
        c(n)=1;
        ex=ex+1;
    else
        if C(x(n),y(n))~=0
            warning('Collision detected between %d and %d', n, C(x(n),y(n)))
        end
        C(x(n),y(n))=n;
    end
end
if showgraph==1

 surf(1:M+1,1:N+1,C',C');
 view(2);
 colormap([1 1 1; 0 0 0])
 caxis([0 1]);
 axis([1,M+1,1,N+1])
 shading flat
 title(sprintf('Sec = %.0f',(step))) %numsteps shows number of steps
 drawnow;

end
 step=step+1;
 if step>0
     k=[k,length(vehin)];
      av=0;
     nv=0;
     meanS=0;
     cav=0;
     cnv=0;
     for n=vehin
         if nnz(auto==n)==1 
             av=av+v(n);
             cav=cav+1;
         else
             nv=nv+v(n);
             cnv=cnv+1;
         end
         if v(n)==0
             meanS=meanS+1;
         end
     end
     if cav~=0
     AV=[AV;av/cav];
     CAV=[CAV;cav];
     end
     if cnv~=0
     NV=[NV;nv/cnv]; 
     CNV=[CNV;cnv];
     end
     %S=[S,meanS];
     %EX=[EX,ex];
     %for L=1:numel(roads)/3
     %    ccount(L)= nnz(l==L & c==0);
     %end
end
end
avmean=[avmean;mean(AV)];
nvmean=[nvmean;mean(NV)];
totalav=[totalav,mean(CAV)];
totalnv=[totalnv,mean(CNV)];
if mod(simno,3)==0
    avvmean(mod(floor(simno/3-1),5)+1,floor((simno)/15)+1)=mean(avmean);
    avmean=[];
    nvvmean(mod(floor(simno/3-1),5)+1,floor((simno)/15)+1)=mean(nvmean);
    nvmean=[];
    totalavv(mod(floor(simno/3-1),5)+1,floor((simno)/15)+1)=mean(totalav);
    totalav=[];
    totalnvv(mod(floor(simno/3-1),5)+1,floor((simno)/15)+1)=mean(totalnv);
    totalnv=[];
end

end
avvmean=avvmean';
nvvmean=nvvmean';
totalavv=totalavv';
totalnvv=totalnvv';