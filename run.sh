#!/bin/sh

######################################################################
#
# run.sh : LQレギュレータをを用いて倒立振子を動作させる
#
# 動作の概要は次の通り（318行目から391行目）。
# >>> cu ........ （319行目）各種センサの取得および制御量の反映
# ^    v
# ^   kalfltr ... （338行目）カルマンフィルタを用いた角度のノイズ除去
# ^    v
# ^   kalfltr ... （368行目）カルマンフィルタを用いたシステム全体のノイズ除去
# ^    v
# ^<< lqreg ..... （376行目）最適制御による制御量の計算
#
# Written by Shinichi Yanagido (s.yanagido@gmail.com) on 2021-03-14
#
######################################################################


######################################################################
# Set Default Parameters
######################################################################

q='1 1 1000 10'
r='1000000'


######################################################################
# Initial Configuration
######################################################################

# === Initialize shell environment ===================================
set -u
umask 0022
export LC_ALL=C
type command >/dev/null 2>&1 && type getconf >/dev/null 2>&1 &&
export PATH="$(command -p getconf PATH)${PATH+:}${PATH-}"
export UNIX_STD=2003  # to make HP-UX conform to POSIX

# === Set parameters =================================================

# --- Learning
nSamples=200

# --- Serial
device=/dev/ttyACM0
bps=57600

# --- Tamiya sports tire set
m_wheel=0.026 # 質量 (kg)
r_wheel=0.028 # 半径 (m)
# 重心周りの慣性モーメント (kg.m^2)．タイヤの密度が一様であると仮定
I_wheel=$(awk "BEGIN{                                       #
                 CONVFMT = OFMT = \"%.09f\"                 #
                 print 0.5 * $m_wheel * $r_wheel * $r_wheel #
               }"                                           )

# --- Tamiya high power gear box HE
gear_ratio=64.8

# --- Whole
# (質量)/2 (kg)
m_whole=$(awk 'BEGIN{CONVFMT=OFMT="%.09f"; print 0.686 / 2}')
# (重心周りの慣性モーメント)/2 (kg.m^2)，下式は二点吊り法による計算
I_whole=$(awk "BEGIN{                                                           #
                 CONVFMT = OFMT = \"%.09f\"                                     #
                 print $m_whole * 9.8 * 0.1275 * 0.0425                         \
                   / 4.0 / (atan2(0,-0)^2) / ((19.0/(31.018-0.853))^2) / 0.8375 #
               }"                                                               )
# 全体の重心とタイヤの重心の距離 (m)
dist_gwheel=0.0727
# 全体の重心とボディの重心の距離 (m)
dist_gbody=0.0020
# タイヤの重心とボディの重心の距離 (m)
r_pendulum=$(awk "BEGIN{CONVFMT=OFMT=\"%.09f\"; print $dist_gwheel + $dist_gbody}")

# --- Body
# (質量)/2 (kg)
m_pendulum=$(awk "BEGIN{CONVFMT=OFMT=\"%.09f\"; print $m_whole - $m_wheel}")
# (タイヤ軸周りの慣性モーメント)/2 (kg.m^2)
I_pendulum=$(awk "BEGIN{                                               #
                    CONVFMT = OFMT = \"%.09f\"                         #
                    print ($I_whole+$m_whole*$dist_gwheel*$dist_gwheel \
                      - $I_wheel)/2                                    #
                  }"                                                   )

# --- Motor (RE-260RA-2670)
Rm=2.4 # resistance
kt=0.0018 # The torque constant (N.m/A)
kb=0.0024 # The back electromotive force constant (V.s/rad)
m_rotator=0.010 # The mass of rotator (kg)
r_rotator=0.0075 # The radius of rotator (m)
motor_offset=0.17 # 摩擦トルクの影響による実行的なモータ電圧のオフセット(V)
# The moment of inertia of the rotator
Im=$(awk "BEGIN{                                       #
            CONVFMT = OFMT = \"%.09f\"                 #
            print 0.5*$m_rotator*$r_rotator*$r_rotator #
          }"                                           )

# --- Rotary Encoder
rotary_encoder_resolution=100 # pulse/rotate

# --- System noise
voltage_error=0.1
voltage_variance=$(awk "BEGIN{                                  #
                          CONVFMT = OFMT = \"%.09f\"            #
                          print $voltage_error * $voltage_error #
                        }"                                      )

# === Define the functions for printing usage and error message ======
print_usage_and_exit () {
  cat <<-USAGE 1>&2
	Usage   : ${0##*/} [-q '<q1> <q2> <q3> <q4>'] [-r <r>]
	Args    : q1,q2,q3,q4 ... 最適制御で用いる状態ベクトルの係数。指定されない場合、q='$q'
	          r ............. 最適制御で用いる入力ベクトルの係数。指定されない場合、r='$r'
	Version : 2020-07-17 14:07:55 JST
	USAGE
  exit 1
}
exit_trap() {
  set -- ${1:-} $?  # $? is set as $1 if no argument given
  trap '-' EXIT HUP INT QUIT PIPE ALRM TERM
  [ -d "${Tmp:-}" ] && rm -rf "${Tmp%/*}/_${Tmp##*/_}"
  exit $1
}
warning() {
  ${1+:} false && echo "${0##*/}: $1" 1>&2
}
error_exit() {
  ${2+:} false && echo "${0##*/}: $2" 1>&2
  exit $1
}

# === Detect home directory of this app. and define more =============
Homedir="$(d=${0%/*}/; [ "_$d" = "_$0/" ] && d='./'; cd "$d.."; pwd)"
PATH="$Homedir/BIN:$Homedir/TOOL:$PATH" # for additional command
. "$Homedir/CONFIG/COMMON.SHLIB"        # read config file


######################################################################
# Parse Arguments
######################################################################

# === Print usage and exit if one of the help options is set =========
case "$# ${1:-}" in
  '1 -h'|'1 --help'|'1 --version') print_usage_and_exit;;
esac

# === Read options ===================================================
while :; do
  case "${1:-}" in
    -q)     q="$2"
            shift 2
            ;;
    -r)     r="$2"
            shift 2
            ;;
    --|-)   break
            ;;
    --*|-*) error_exit 1 'Invalid option'
            ;;
    *)      break
            ;;
  esac
done


######################################################################
# Main Routine
######################################################################

#=== 初期処理 ========================================================

#--- tmpディレクトリに名前付きパイプを作成
trap 'exit_trap' EXIT HUP INT QUIT PIPE ALRM TERM
Tmp=`mktemp -d -t "_${0##*/}.$$.XXXXXXXXXXX"` || error_exit 1 'Failed to mktemp'
mkfifo $Tmp/named_pipe || error_exit 1 'Failed to mkfifo'

#--- データの学習
warning "Initializing"
statistics=$( (printf ' s '; cat $Tmp/named_pipe >/dev/null; printf ' s ~.') |
             cu -s $bps -l "$device"                              |
             linets -9d                                           |
             ptw cut -d ' ' -f 2-                                 |
             ptw tail -n +10                                      |
             ptw awk 'BEGIN{                                      #
                        CONVFMT = OFMT = "%.09f"                  #
                      }                                           #
                      {                                           #
                        print $1, -atan2($4,$3)*57.29578, 2.0*$5  #
                      }'                                          |
             (head -n $nSamples; echo >$Tmp/named_pipe)           |
             awk 'BEGIN{                                          #
                    CONVFMT = OFMT = "%.09f"                      #
                  }                                               #
                  {                                               #
                    for (i=1; i<=NF; i++) {                       #
                      sum[i]+=$i                                  #
                      sqsum[i]+=$i*$i                             #
                    }                                             #
                  }                                               #
                  END{                                            #
                    for(i=1; i<=length(sum); i++)                 #
                      printf "%f %f ", sum[i]/NR                  \
                                     , sqsum[i]/NR-(sum[i]/NR)^2  #
                    print ""                                      #
                  }'                                              )
feedback_rate=0.01
theta_mean=$(        echo $statistics | cut -d ' ' -f 3)
theta_variance=$(    echo $statistics | cut -d ' ' -f 4)
theta_dot_mean=$(    echo $statistics | cut -d ' ' -f 5)
theta_dot_variance=$(echo $statistics | cut -d ' ' -f 6)
warning "Initialization done"

#--- パラメータの計算
# 行列A
a11_temp=$(awk "BEGIN{                                                 #
                  CONVFMT = OFMT = \"%.09f\"                           #
                  print ($m_wheel + $m_pendulum) * $r_wheel * $r_wheel \
                        + 2 * $m_pendulum * $r_wheel * $r_pendulum     \
                        + $m_pendulum * $r_pendulum * $r_pendulum      \
                        + $I_pendulum + $I_wheel                       #
                }"                                                     )
a12_temp=$(awk "BEGIN{                                                 #
                  CONVFMT = OFMT = \"%.09f\"                           #
                  print ($m_wheel + $m_pendulum) * $r_wheel * $r_wheel \
                        + $m_pendulum * $r_wheel * $r_pendulum         \
                        + $I_wheel                                     #
                }"                                                     )
a21_temp=$(awk "BEGIN{                                                 #
                  CONVFMT = OFMT = \"%.09f\"                           #
                  print ($m_wheel + $m_pendulum) * $r_wheel * $r_wheel \
                        + $m_pendulum * $r_wheel * $r_pendulum         \
                        + $I_wheel                                     #
                }"                                                     )
a22_temp=$(awk "BEGIN{                                                 #
                  CONVFMT = OFMT = \"%.09f\"                           #
                  print ($m_wheel + $m_pendulum) * $r_wheel * $r_wheel \
                        + $I_wheel                                     \
                        + $gear_ratio * $gear_ratio * $Im              #
                }"                                                     )
det=$(awk "BEGIN{                                            #
             CONVFMT = OFMT = \"%.09f\"                      #
             print $a11_temp*$a22_temp - $a12_temp*$a21_temp #
           }"                                                )
a11=$(awk "BEGIN{CONVFMT=OFMT=\"%.09f\"; print  $a22_temp/$det}")
a12=$(awk "BEGIN{CONVFMT=OFMT=\"%.09f\"; print -$a12_temp/$det}")
a21=$(awk "BEGIN{CONVFMT=OFMT=\"%.09f\"; print -$a21_temp/$det}")
a22=$(awk "BEGIN{CONVFMT=OFMT=\"%.09f\"; print  $a11_temp/$det}")
A21=$(awk "BEGIN{                                         #
             CONVFMT = OFMT = \"%.09f\"                   #
             print $a11 * $m_pendulum * 9.8 * $r_pendulum #
           }"                                             )
A24=$(awk "BEGIN{                                                          #
             CONVFMT = OFMT = \"%.09f\"                                    #
             print -1 * $a12 * $gear_ratio * $gear_ratio * $kt * $kb / $Rm #
           }"                                                              )
A41=$(awk "BEGIN{                                         #
             CONVFMT = OFMT = \"%.09f\"                   #
             print $a21 * $m_pendulum * 9.8 * $r_pendulum #
           }"                                             )
A44=$(awk "BEGIN{                                                          #
             CONVFMT = OFMT = \"%.09f\"                                    #
             print -1 * $a22 * $gear_ratio * $gear_ratio * $kt * $kb / $Rm #
           }"                                                              )
A="0 1 0 0; $A21 0 0 $A24; 0 0 0 1; $A41 0 0 $A44"

# 行列B
B2=$(awk "BEGIN{CONVFMT=OFMT=\"%.09f\"; print $a12 * $gear_ratio * $kt / $Rm}")
B4=$(awk "BEGIN{CONVFMT=OFMT=\"%.09f\"; print $a22 * $gear_ratio * $kt / $Rm}")
B="0; $B2; 0; $B4"

# 観測の共分散行列
deg_rad_coeff=$(awk 'BEGIN{                        #
                       CONVFMT = OFMT = "%.09f"    #
                       pi = atan2(0,-0)            #
                       print (pi*pi)/(180.0*180.0) #
                     }'                            )
W11=$(awk "BEGIN{CONVFMT=OFMT=\"%.09f\"; print $theta_variance     * $deg_rad_coeff}")
W22=$(awk "BEGIN{CONVFMT=OFMT=\"%.09f\"; print $theta_dot_variance * $deg_rad_coeff}")
encoder_error=$(awk "BEGIN{                                              #
                       CONVFMT = OFMT = \"%.09f\"                        #
                       pi = atan2(0,-0)                                  #
                       print 0.1*2.0*pi/(4.0*$rotary_encoder_resolution) #
                     }"                                                  )
W33=$(awk "BEGIN{                                  #
             CONVFMT = OFMT = \"%.09f\"            #
             print $encoder_error * $encoder_error #
           }"                                      )
encoder_rate_error=$(awk "BEGIN{                                  #
                            CONVFMT = OFMT = \"%.09f\"            #
                            print $encoder_error / $feedback_rate #
                          }"                                      )
W44=$(awk "BEGIN{                                            #
             CONVFMT = OFMT = \"%.09f\"                      #
             print $encoder_rate_error * $encoder_rate_error #
           }"                                                )
W="$W11 0 0 0; 0 $W22 0 0; 0 0 $W33 0; 0 0 0 $W44"

# パラメータの出力
# warning "$statistics"
warning "feedback_rate      = $feedback_rate"
warning "theta_mean         = $theta_mean"
warning "theta_variance     = $theta_variance"
warning "theta_dot_mean     = $theta_dot_mean"
warning "theta_dot_variance = $theta_dot_variance"
warning "Diag(Q) = ($q)"
warning "Diag(R) = ($r)"

#=== 制御 ============================================================

#--- コマンドの立ち上げ用の名前付きパイプ作成 ------------------------
mkfifo $Tmp/pipe_lqreg_init || error_exit 1 'Failed to mkfifo'

#--- 制御開始 --------------------------------------------------------
#--- デバイス                                                 #
(cat $Tmp/pipe_lqreg_init >/dev/null                          #
 cu -s $bps -l "$device" <$Tmp/named_pipe                   ) |
ptw awk '{print 0.01, $0}'                                    |
ptw tail -n +10                                               |
# 1:dt 2-4:acc 5-7:gyro 8:rot 9:motor-speed                   #
#--- 姿勢の推定                                               #
ptw awk '                                                     #
  BEGIN{                                                      #
    CONVFMT = OFMT = "%.09f"                                  #
    pi = atan2(0,-0)                                          #
    r2dcoeff = 180.0/pi                                       #
  }                                                           #
  {                                                           #
    print $1,                                                 \
          -atan2($4, $3) * r2dcoeff,                          \
          $5,                                                 \
          $8,                                                 \
          -$9                                                 #
  }'                                                          |
# 1:dt 2:deg 3:x-gyro 4:rot 5:motor-speed                     #
ptw kalfltr -A '0 -1; 0 0'                                    \
            -B '1; 0'                                         \
            -C '1 0'                                          \
            -x "0; $theta_dot_mean"                           \
            -P "1 0; 0 $theta_dot_variance"                   \
            -U "$theta_dot_variance"                          \
            -W "$theta_variance"                              |
# 1:dt 2:est-deg 3:x-gyro-bias 4:deg 5:x-gyro 6:rot           #
# 7:motor-speed                                               #
#--- システム全体の推定                                       #
ptw awk '                                                     #
  BEGIN{                                                      #
    CONVFMT = OFMT = "%.09f"                                  #
    pi = atan2(0,-0)                                          #
    d2rcoeff = pi/180.0                                       #
    p2vcoeff = 5.0/255.0                                      #
    e2rcoeff = 2.0*pi/(4.0*'"$rotary_encoder_resolution"')    #
  }                                                           #
  {                                                           #
    enc += $6                                                 #
    print                                                     \
      $1,                                                     \
      ($2 - '"$theta_mean"') * d2rcoeff,                      \
      ($5 - $3) * d2rcoeff,                                   \
      enc * e2rcoeff,                                         \
      ($1!=0.0) ? $6 * e2rcoeff / $1 : 0.0,                   \
      $7 * p2vcoeff                                           #
  }'                                                          |
# 1:dt 2:est-rad 3:pendulum-rad/sec 4:wheel-rad               #
# 5:wheel-rad/sec 6:voltage                                   #
ptw kalfltr                                                   \
  -A "$A" -B "$B" -C '1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1'     \
  -P '0.0001 0 0 0; 0 0.0001 0 0; 0 0 0.0001 0; 0 0 0 0.0001' \
  -U $voltage_variance -W "$W"                                |
ptw cut -d ' ' -f -5                                          |
# 1:dt 2:est-rad 3:est-pend-rad/sec 4:est-wheel-rad           #
# 5:est-wheel-rad/sec                                         #
#--- 制御量の計算                                             #
ptw lqreg -A "$A" -B "$B" -q "$q" -r "$r" -L 0.00851516       \
          -o $Tmp/pipe_lqreg_init                             |
# 1:dt 2:voltage                                              #
ptw awk '                                                     #
  BEGIN{                                                      #
    CONVFMT = OFMT = "%.09f"                                  #
    v2pcoeff = 255.0/5.0                                      #
  }                                                           #
  {                                                           #
    if ($2 > 0.0) {                                           #
      $2 += '"$motor_offset"'                                 #
    } else if ($2 < 0.0) {                                    #
      $2 -= '"$motor_offset"'                                 #
    }                                                         #
    print int($2 * v2pcoeff)                                  #
  }'                                                          >$Tmp/named_pipe


######################################################################
# Finish
######################################################################
exit 0
