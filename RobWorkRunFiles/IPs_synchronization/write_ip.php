<?php
$myfile = fopen("ip_".$_GET["device"].".txt", "w") or die("Unable to open file!");
fwrite($myfile, $_GET["ip"]);
fclose($myfile);
?>
