function [vs, ws, stop] = controlParking(q, goalPose, goalId, vehicleId, collision, inside, map)

% Trenutni polozaj robota
x = q(1);
y = q(2);
fi = q(3);
gama = q(4);

% Ciljni polozaj
xc = goalPose(1);
yc = goalPose(2);
fic = goalPose(3);

% Parametri robota
D = 0.1207;

% Zapomnimo si fazo
persistent faza podfaza indeks_poti

if isempty(faza)
    faza = 1;
    podfaza = 1;
    indeks_poti = 1;
end

vs = 0;
ws = 0;
stop = false;

% ========================================
% P4 - VZVRATNO PARKIRANJE PO KRIVULJI
% ========================================

if goalId == 4

    % Zacetek poti v sredinskem prehodu
    x0 = xc + 0.30;
    y0 = yc + 0.31;

    % Kontrolne tocke Bezierjeve krivulje
    x1 = xc + 0.30;
    y1 = yc + 0.15;

    x2 = xc + 0.20;
    y2 = yc;

    % Izracun krivulje
    t = linspace(0,1,101);

    pot_x = (1-t).^3*x0 + ...
        3*(1-t).^2.*t*x1 + ...
        3*(1-t).*t.^2*x2 + t.^3*xc;

    pot_y = (1-t).^3*y0 + ...
        3*(1-t).^2.*t*y1 + ...
        3*(1-t).*t.^2*y2 + t.^3*yc;

    % FAZA 1 - prihod do zacetka krivulje
    if faza == 1

        dx = x0-x;
        dy = y0-y;

        razdalja = hypot(dx,dy);
        smer = atan2(dy,dx);
        napaka = wrapToPi(smer-fi);

        vs = min(0.16,0.8*razdalja);

        if abs(napaka) > pi/2
            vs = -vs;
            napaka = wrapToPi(napaka-pi);
        end

        omega = 2*napaka;

        if abs(vs) > 0.005
            vrednost = D*omega/vs;
            vrednost = max(-0.95,min(0.95,vrednost));
            gama_ref = asin(vrednost);
        else
            gama_ref = 0;
        end

        ws = 5*wrapToPi(gama_ref-gama) ...
             + vs*sin(gama)/D;

        if razdalja < 0.005
            faza = 2;
        end

    % FAZA 2 - zasuk koles za obracanje
    elseif faza == 2

        vs = 0;
        ws = 4*wrapToPi(pi/2-gama);

        if abs(wrapToPi(pi/2-gama)) < 0.01
            faza = 3;
        end

    % FAZA 3 - obrnemo robota navzgor
    elseif faza == 3

        napaka = wrapToPi(pi/2-fi);

        vs = max(-0.07,min(0.07,1.5*D*napaka));

        ws = vs*sin(gama)/D ...
             + 4*wrapToPi(pi/2-gama);

        if abs(napaka) < deg2rad(0.2)
            faza = 4;
        end

    % FAZA 4 - poravnamo kolesa
    elseif faza == 4

        vs = 0;
        ws = 4*wrapToPi(-gama);

        if abs(gama) < 0.008
            faza = 5;
        end

    % FAZA 5 - vzvratno po krivulji
    elseif faza == 5

        % Najblizja tocka na krivulji
        razdalje = ...
            (pot_x(indeks_poti:end)-x).^2 + ...
            (pot_y(indeks_poti:end)-y).^2;

        [~, mesto] = min(razdalje);

        indeks_poti = indeks_poti + mesto - 1;

        % Referencna tocka malo naprej po krivulji
        naslednja = min(indeks_poti+10,101);

        xr = pot_x(naslednja);
        yr = pot_y(naslednja);

        dx = xr-x;
        dy = yr-y;

        razdalja = hypot(dx,dy);

        % Bocna razdalja v koordinatah robota
        bocna = -sin(fi)*dx + cos(fi)*dy;

        % Pure Pursuit - krivina
        krivina = 2*bocna/max(razdalja^2,0.0016);

        % Omejitev krivine
        krivina = max(-7,min(7,krivina));

        % Razdalja do koncnega cilja
        napaka_polozaja = hypot(xc-x,yc-y);
        napaka_kota = abs(wrapToPi(fic-fi));

        % Vzvratna hitrost
        vs = -min(0.10,max(0.004, ...
             0.7*napaka_polozaja));

        % Blizu cilja pocasneje
        if napaka_polozaja < 0.06
            vs = -min(0.04,max(0.004, ...
                 0.5*napaka_polozaja));

            % Tocno sledenje koncnemu cilju
            dx = xc-x;
            dy = yc-y;

            razdalja = hypot(dx,dy);
            bocna = -sin(fi)*dx + cos(fi)*dy;

            krivina = 2*bocna/max(razdalja^2,0.0016);

            % Dodatno popravljanje orientacije
            napaka_fi = wrapToPi(fic-fi);
            krivina = krivina - 2*napaka_fi;

            krivina = max(-7,min(7,krivina));
        end

        % Izracun zelenega zasuka koles
        vrednost = D*krivina;
        vrednost = max(-0.95,min(0.95,vrednost));

        gama_ref = asin(vrednost);

        % Regulacija zasuka
        ws = 5*wrapToPi(gama_ref-gama) ...
             + vs*sin(gama)/D;

        % Preverjanje koncnega polozaja
        if napaka_polozaja < 0.001 && ...
           napaka_kota < deg2rad(0.5) && inside

            stop = true;
            vs = 0;
            ws = 0;
        end
    end

% ========================================
% P5 IN P6 - OBRACANJE OKOLI ZADNJE OSI
% ========================================

elseif goalId == 5 || goalId == 6

    % FAZA 1 - prihod nad parkirisce
    if faza == 1

        if podfaza == 1
            xr = xc;
            yr = 0.82;
        else
            xr = xc;
            yr = yc + 0.30;
        end

        dx = xr-x;
        dy = yr-y;

        razdalja = hypot(dx,dy);
        smer = atan2(dy,dx);
        napaka = wrapToPi(smer-fi);

        vs = min(0.16,0.8*razdalja);

        if abs(napaka) > pi/2
            vs = -vs;
            napaka = wrapToPi(napaka-pi);
        end

        omega = 2*napaka;

        if abs(vs) > 0.005
            gama_ref = atan(D*omega/vs);
        else
            gama_ref = 0;
        end

        ws = 5*wrapToPi(gama_ref-gama) ...
             + vs*sin(gama)/D;

        if podfaza == 1 && razdalja < 0.005
            podfaza = 2;
        elseif podfaza == 2 && razdalja < 0.003
            faza = 2;
        end

    % FAZA 2 - zasuk sprednjih koles
    elseif faza == 2

        vs = 0;
        ws = 4*wrapToPi(pi/2-gama);

        if abs(wrapToPi(pi/2-gama)) < 0.005
            faza = 3;
        end

    % FAZA 3 - obrat robota navzgor
    elseif faza == 3

        napaka = wrapToPi(pi/2-fi);

        vs = max(-0.08,min(0.08,1.8*D*napaka));

        ws = vs*sin(gama)/D ...
             + 4*wrapToPi(pi/2-gama);

        if abs(napaka) < deg2rad(0.15)
            faza = 4;
        end

    % FAZA 4 - poravnava koles
    elseif faza == 4

        vs = 0;
        ws = 4*wrapToPi(-gama);

        if abs(gama) < 0.005
            faza = 5;
        end

    % FAZA 5 - vzvratno do ciljne osi
    elseif faza == 5

        dx = xc-x;
        dy = yc-y;

        vs = max(-0.10,min(0.10,0.8*dy));

        fi_ref = pi/2 + ...
            max(-0.20,min(0.20,3*dx));

        napaka = wrapToPi(fi_ref-fi);
        omega = 2*napaka;

        if abs(vs) > 0.002
            gama_ref = atan(D*omega/vs);
        else
            gama_ref = 0;
        end

        ws = 5*wrapToPi(gama_ref-gama) ...
             + vs*sin(gama)/D;

        if hypot(dx,dy) < 0.0005 && ...
           abs(wrapToPi(pi/2-fi)) < deg2rad(0.15)
            faza = 6;
        end

    % FAZA 6 - zasuk koles
    elseif faza == 6

        vs = 0;
        ws = 4*wrapToPi(pi/2-gama);

        if abs(wrapToPi(pi/2-gama)) < 0.005
            faza = 7;
        end

    % FAZA 7 - koncna orientacija
    elseif faza == 7

        napaka = wrapToPi(fic-fi);

        vs = max(-0.07,min(0.07,1.5*D*napaka));

        ws = vs*sin(gama)/D ...
             + 4*wrapToPi(pi/2-gama);

        if abs(napaka) < deg2rad(0.15)
            faza = 8;
        end

    % FAZA 8 - poravnava koles
    elseif faza == 8

        vs = 0;
        ws = 4*wrapToPi(-gama);

        if abs(gama) < 0.005
            faza = 9;
        end

    % FAZA 9 - preverjanje
    elseif faza == 9

        napaka_polozaja = hypot(xc-x,yc-y);
        napaka_kota = abs(wrapToPi(fic-fi));

        vs = 0;
        ws = 0;

        if napaka_polozaja < 0.001 && ...
           napaka_kota < deg2rad(0.5) && inside
            stop = true;
        end
    end

% ========================================
% OSTALA PARKIRISCA
% ========================================

else

    smer_park = 1;

    stransko = abs(cos(fic)) > 0.9 && yc > 0.17;

    if stransko
        if xc < map.width/2
            if cos(fic) > 0
                smer_park = -1;
            end
        else
            if cos(fic) < 0
                smer_park = -1;
            end
        end
    end

    odmik = 0.25;

    if smer_park == -1
        odmik = 0.32;
    end

    x_vmes = xc-smer_park*odmik*cos(fic);
    y_vmes = yc-smer_park*odmik*sin(fic);

    % FAZA 1 - prihod do vmesne tocke
    if faza == 1

        if stransko && smer_park == -1 && podfaza == 1

            if xc < map.width/2
                xr = 0.40;
            else
                xr = 1.00;
            end

            yr = 0.70;
        else
            xr = x_vmes;
            yr = y_vmes;
        end

        dx = xr-x;
        dy = yr-y;

        razdalja = hypot(dx,dy);
        smer = atan2(dy,dx);
        napaka = wrapToPi(smer-fi);

        vs = min(0.18,0.8*razdalja);

        if abs(napaka) > pi/2
            vs = -vs;
            napaka = wrapToPi(napaka-pi);
        end

        omega = 2*napaka;

        if abs(vs) > 0.005
            gama_ref = atan(D*omega/vs);
        else
            gama_ref = 0;
        end

        ws = 5*wrapToPi(gama_ref-gama) ...
             + vs*sin(gama)/D;

        if stransko && smer_park == -1 && podfaza == 1

            if razdalja < 0.02
                podfaza = 2;
            end

        elseif razdalja < 0.005
            faza = 2;
        end

    % FAZA 2 - zasuk koles
    elseif faza == 2

        vs = 0;
        ws = 4*wrapToPi(pi/2-gama);

        if abs(wrapToPi(pi/2-gama)) < 0.01
            faza = 3;
        end

    % FAZA 3 - obrnemo robota
    elseif faza == 3

        napaka = wrapToPi(fic-fi);

        vs = max(-0.12,min(0.12,1.5*D*napaka));

        ws = vs*sin(gama)/D ...
             + 4*wrapToPi(pi/2-gama);

        if abs(napaka) < deg2rad(0.3)
            faza = 4;
        end

    % FAZA 4 - poravnamo kolesa
    elseif faza == 4

        vs = 0;
        ws = 4*wrapToPi(-gama);

        if abs(gama) < 0.01
            faza = 5;
        end

    % FAZA 5 - voznja do cilja
    elseif faza == 5

        dx = xc-x;
        dy = yc-y;

        razdalja = dx*cos(fic)+dy*sin(fic);

        vs = max(-0.12,min(0.12,0.8*razdalja));

        ws = 5*wrapToPi(-gama);

        napaka_polozaja = hypot(dx,dy);
        napaka_kota = abs(wrapToPi(fic-fi));

        if napaka_polozaja < 0.001 && ...
           napaka_kota < deg2rad(0.5) && inside

            stop = true;
            vs = 0;
            ws = 0;
        end
    end
end

end
