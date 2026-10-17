function [u, link] = hil_bridge(eta, nu, eta_d, nu_d)
%HIL_BRIDGE  MATLAB Function block body: one HIL round trip per step.
%
%   Wire this block into rov_hil_host.slx in place of the controller:
%
%       eta    <- Euler Transformation1   (6x1, current pose)
%       nu     <- Integrator              (6x1, current body velocity)
%       eta_d  <- Mux of x_d..yaw_d       (6x1, reference pose)
%       nu_d   <- constant 0              (unused by the controller)
%
%       u      -> Inverse Thruster Matrix
%       link   -> To Workspace (diagnostics)
%
%   The real work happens in hil_bridge_host.m, which is executed by the
%   MATLAB interpreter (coder.extrinsic) so that it can own a persistent
%   serialport object.

%#codegen
coder.extrinsic('hil_bridge_host');

u    = zeros(6,1);
link = zeros(1,1);

[u, link] = hil_bridge_host(eta, nu, eta_d, [0;0;0;0;0;0]);
end
