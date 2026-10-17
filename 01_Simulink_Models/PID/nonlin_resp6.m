figure()
subplot(2,3,1)
plot(t,eout(:,1),'LineWidth',1.5)
xlabel('time（s）')
ylabel('x(m)')

subplot(2,3,2)
plot(t,eout(:,2),'LineWidth',1.5)
title('Position Response')
xlabel('time（s）')
ylabel('y(m)')

subplot(2,3,3)
plot(t,eout(:,3),'LineWidth',1.5)
xlabel('time（s）')
ylabel('z(m)')

subplot(2,3,4)
plot(t,eout(:,4),'LineWidth',1.5)
xlabel('time（s）')
ylabel('\phi(deg)')

subplot(2,3,5)
plot(t,eout(:,5),'LineWidth',1.5)
xlabel('time（s）')
ylabel('\theta(deg)')

subplot(2,3,6)
plot(t,eout(:,6),'LineWidth',1.5)
xlabel('time（s）')
ylabel('\eta(deg)')


%--------------------------
figure()
subplot(2,3,1)
plot(t,bout(:,1),'LineWidth',1.5)
xlabel('时间（s）')
ylabel('u(m/s)')

subplot(2,3,2)
plot(t,bout(:,2),'LineWidth',1.5)
title('Velocity Response')
xlabel('时间（s）')
ylabel('v(m/s)')

subplot(2,3,3)
plot(t,bout(:,3),'LineWidth',1.5)
xlabel('Time(sec)')
ylabel('w(m/s)')

subplot(2,3,4)
plot(t,bout(:,4),'LineWidth',1.5)
xlabel('Time(sec)')
ylabel('p(rad/s)')

subplot(2,3,5)
plot(t,bout(:,5),'LineWidth',1.5)
xlabel('Time(sec)')
ylabel('q(rad/s)')

subplot(2,3,6)
plot(t,bout(:,6),'LineWidth',1.5)
xlabel('Time(sec)')
ylabel('r(rad/s)')
