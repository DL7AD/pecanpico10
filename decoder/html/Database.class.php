<?php
require_once "Tracker.class.php";

class Database extends mysqli {
	private static $instance = null;

	public function __construct() {
		if(self::$instance != NULL) {
			die("Fick dich!");
		}
		$this->con = parent::__construct("localhost", "decoder", "decoder", "decoder");
		if(mysqli_connect_errno())
			die(mysql_error());
	}

	public function __destruct() {
		$this->close();
	}

	public static function getInstance() {
		if(self::$instance == null)
			self::$instance = new Database();
		return self::$instance;
	}

	public function close() {
		if(is_null($this->con))
			return;
		parent::close();
		$this->con = null;
	}

	public function getTracker() {
		$tracker = array();

		$query = $this->query("
			SELECT `call`,MAX(`rxtime`) FROM (
				(
					SELECT `call`,MAX(`rxtime`) AS `rxtime`
					FROM `position`
					GROUP BY `call`
				)
				UNION
				(
					SELECT `call`,MAX(`rxtime`) AS `rxtime`
					FROM `image`
					GROUP BY `call`
				)
			) tu
			GROUP BY `call`
		");
		while($row = $query->fetch_assoc())
			$tracker[] = new Tracker($row['call']);

		return $tracker;
	}
}
?>

