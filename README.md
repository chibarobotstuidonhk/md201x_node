# md201x_node

launchファイルにて
```
<node pkg="md201x_node" type="md201x_node" name="motor1" output="screen">
      <param name="bid" value="4a0" />
      <param name="name" value="motor1" />
</node>
```
と，bidと使用するモータを表す名前をつけてあげること．
雑な作りなのでlaunchでnameをつけてあげないと落ちます．(まあ上記以外の使い方はしないと思いますが…)
 
