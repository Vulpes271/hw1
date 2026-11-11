function testParking(randomRuns)
% Ponovljivi testi vseh primerov iz main.pdf z vkljucenim sumom.
% Rezultati so enaki obliki podatkov iz simParking, dodana sta phi/gamma.
if nargin < 1, randomRuns = 1; end
qA = [.3;.9;0;0];
qB = [.9;.9;0;0];
qC = [.4;.9;pi;0];
qD = [.8;.7;pi/2;0];
qS = [.6;.9;0;0];
starts = {qA,qB,qC,qD,qA,qA,qA};
targets = [11,11,11,11,1,7,5];
for j = 1:7
    rng(100+j);
    fprintf('\nNaloga 1, primer %d\n',j);
    data = runParkingCase(starts{j},targets(j),false,1);
    store(sprintf('ex1case%d',j),data);
end
sequences = {[1,8,10,3,6],[2,9,7,4], ...
    [1,3,10,8,6,2,7,9,4,5],[11,1,2,9,10,5,3,8,7,4,6]};
for limited = [false,true]
    exercise = 2+limited;
    for j = 1:4
        rng(100*exercise+j);
        fprintf('\nNaloga %d, primer %d\n',exercise,j);
        data = runParkingCase(qS,sequences{j},limited,1);
        store(sprintf('ex%dcase%d',exercise,j),data);
    end
end
for j = 1:randomRuns
    rng(400+j);
    data = runParkingCase([],0,false,1);
    store(sprintf('ex1random%d',j),data);
    for limited = [false,true]
        rng(500+100*limited+j);
        data = runParkingCase([],zeros(1,11),limited,1);
        store(sprintf('ex%drandom%d',2+limited,j),data);
    end
end
fprintf('\nVsi testi so uspeli.\n');
end
