require 'pry'

options = '--p2-max-iteration=150 --p2-init-box-size=25 --p2-box-expand-size=20 --overflow-threshold=100 --p3-max-iteration=20 --p3-init-box-size=15 --p3-box-expand-size=15'

benchs = {
    adaptec1: 'adaptec1.capo70.3d.35.50.90.gr',
    adaptec2: 'adaptec2.mpl60.3d.35.20.100.gr',
    adaptec3: 'adaptec3.dragon70.3d.30.50.90.gr',
    adaptec4: 'adaptec4.aplace60.3d.30.50.90.gr',
    adaptec5: 'adaptec5.mfar50.3d.50.20.100.gr',
    bigblue1: 'bigblue1.capo60.3d.50.10.100.gr',
    newblue1: 'newblue1.ntup50.3d.30.50.90.gr',
    newblue2: 'newblue2.fastplace90.3d.50.20.100.gr',
    newblue3: 'newblue3.kraftwerk80.3d.40.50.90.gr',
    newblue4: 'newblue4.mpl50.3d.40.10.95.gr'
}

benchs.each do |bench|
    folder = bench[0].to_s
    file = bench[0].to_s + "_" + Time.now.strftime("%d%m%Y%H%M").to_s + "OUTPUT.gr"
    input = bench[1].to_s
    system("./Debug/ISPD2008-NTHU-R-CodeRelease-Updated --input=benchs/#{input} --output=outputs/#{folder}/#{file} #{options} --monotonic-routing=1 > outputs/#{folder}/log_#{Time.now.strftime("%d%m%Y%H%M").to_s}")
end
