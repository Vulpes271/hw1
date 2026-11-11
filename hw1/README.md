Rešitve vseh treh nalog so v `controlParking.m` in `controlParkingLim.m`.
Obe uporabljata skupni `parkingControl.m` in `parkingPlan.m`.
Ni posebnih vej glede na številko parkirišča ali vozila.

V MATLAB-u odpri to mapo in zaženi:

```matlab
simParking([0.3; 0.9; 0; 0], 11);
simParkingLim([0.6; 0.9; 0; 0], [1, 8, 10, 3, 6]);
```

`homework.mlx` vsebuje razlago, posamezne primere, grafe in tabele.
Za ponovitev vseh predpisanih primerov ter enega naključnega zapolnjevanja
parkirišča za vsako različico uporabi:

```matlab
testParking(1)
```

Testi uporabljajo izvirni `updateVehicle` z `ode45`, `VehicleConstraints`
in `Map`. Funkcija `runParkingCase.m` izpusti sprotne grafične posodobitve,
zato je hitrejša od animacije; podatke nariše `parkingResults.m`.
Preverja dejansko končno napako, celoten obris, omejitve koles in zasuka.
Rezultati se shranijo v datoteke `ex*case*.mat` in `ex*random*.mat`.
`originalSimulatorCheck.mat` vsebuje tudi neposredni primerjalni preizkus
z nespremenjenim `simParking`.

Vodenje temelji na premicah in krožnih lokih, predkrmiljenju ukrivljenosti
ter povratni vezavi prečne in kotne napake. Hibridni A* iz gradiva predavanj
samodejno izbere vmesne lege ter upošteva že parkirana vozila.
Za oceno razdalje okoli ovir uporablja Dijkstrov postopek na ravninski mreži.
Blizu parkirišča uporablja mrežo 2 mm in 1°; med bližnjima legama z različno
orientacijo preveri tudi tri tangentne loke z menjavo smeri.
Iskanje začne pri parkirišču; pri izvedbi se pot in smer vožnje obrneta.
Pred spremembo ukrivljenosti robot nastavi krmilni kot med mirovanjem.

Filter uporablja kinematiko in privzete omejitve simulatorja
(`T = 0.01 s`, `vMax = 0.3 m/s`, `aMax = 1 m/s²`). Za kodo niso potrebni
dodatni MATLAB toolboxi. Izvirne datoteke simulatorja ostanejo nespremenjene.

Pred oddajo dopolni priimek v uvodu poročila.
Poročilo izrecno navede pomoč OpenAI Codex pri kodi,
preverjanju in razlagi. Po morebitnih spremembah izberi Run All, shrani
`homework.mlx` in šele nato izvedi `prepare`, da ZIP vsebuje shranjene izpise.


Dodatno neposredno preverjanje v MATLAB-u:
Vseh 18 primerov oziroma 90 parkiranj je bilo izvedenih tudi z originalnima
simParking in simParkingLim (noise = 1, animate = 0). Uspelo je vseh 90
parkiranj brez zaznanih trkov. Najvecja koncna polozajna napaka je bila
0.762720 mm, najvecja kotna napaka pa 0.447463 stopinje, kar je pod mejama
1 mm in 0.5 stopinje. Preverjene so tudi omejitve hitrosti in pospeskov koles.
Pri nalogah 1 in 2 se primerjani numericni zapisi povsem ujemajo; pri nalogi
3 se koordinate poti razlikujejo le na ravni numericnega zaokrozevanja
(najvec 6.7e-16 m). Casi parkiranja in zapisi trkov se ujemajo.
Datoteke ex*.mat so rezultati numericnih preizkusov iz porocila;
originalSimulatorCheck.mat vsebuje neposredno primerjavo enega primera.
Osebna razlaga kode in lokalni paketni dnevniki niso del repozitorija.
