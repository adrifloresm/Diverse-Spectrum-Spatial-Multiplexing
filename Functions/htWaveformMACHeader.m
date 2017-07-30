function mpduHeaderBits = htWaveformMACHeader()
% Creates bits for a MAC header  
type = 2;    % Data frame type 2 (10)
subtype = 0; % Data subtype 0 (00)

% Create MPUD header
mac = struct;
frameControl = getFrameControl(type,subtype);
fields = fieldnames(frameControl);
frameControlBits = [];
for i=1:numel(fields)
    frameControlBits = [frameControlBits frameControl.(fields{i})]; 
end
mac.FrameControl = bi2de(reshape(frameControlBits,16,[]).').';% 2 octets
mac.Duration = [5 6];         % Duration of frame for NAV (2 octets)
mac.Address3 = [1 1 1 1 1 1]; %  filtering (6 octets)
mac.bssid = [1];
mac.Address2 = [2 2 2 2 2 2]; % source  address (6 octets)
mac.Address1 = [4 4 4 4 4 4]; % destination address (6 octets)
mac.Sequence = [4 2];         % 2 octets
mpduHeaderBits = octetStruct2bits(mac);
 
end   

% Frame control fields
function frameCtrl = getFrameControl(type,subtype)
    frameCtrl = struct;
    frameCtrl.ProtocolVersion = uint8(de2bi(0,2));
    frameCtrl.Type            = uint8(de2bi(type,2));
    frameCtrl.Subtype         = uint8(de2bi(subtype,4));
    frameCtrl.ToDS            = uint8(0);
    frameCtrl.FromDS          = uint8(0);
    frameCtrl.MoreFragments   = uint8(0);
    frameCtrl.Retry           = uint8(0);
    frameCtrl.PowerManagement = uint8(0);
    frameCtrl.MoreData        = uint8(0);
    frameCtrl.ProtectedFrame  = uint8(0);
    frameCtrl.Order           = uint8(0);
end

% Convert structure with octets to bitstream
function bits = octetStruct2bits(str)
    fnames = fieldnames(str);
    numFields = numel(fnames);
    bits = [];
    for f = 1:numFields
        octets = str.(fnames{f});
        for i=1:numel(octets) 
            bits = [bits; de2bi(double(octets(i)),8).']; %#ok<AGROW>
        end
    end
end